/*
This code was developed by the National Robotics Engineering Center (NREC), part
of the Robotics Institute at Carnegie Mellon University. Its development was
funded by DARPA under the LS3 program and submitted for public release on June
7th, 2012. Release was granted on August, 21st 2012 with Distribution Statement
"A" (Approved for Public Release, Distribution Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.
Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer. Redistributions in binary form must
reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the
distribution. Neither the name of the Carnegie Mellon University nor the names
of its contributors may be used to endorse or promote products derived from this
software without specific prior written permission. THIS SOFTWARE IS PROVIDED BY
THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*-*-C++-*-*/
/**
   @file SpinnakerCamera.cpp
   @author Chad Rockey
   @date July 11, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "spinnaker_camera_driver/SpinnakerCamera.h"

#include <ros/ros.h>

#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <sstream>
#include <string>
#include <typeinfo>

#include "spinnaker_camera_driver/BlueROVCamera.h"

namespace spinnaker_camera_driver {
SpinnakerCamera::SpinnakerCamera()
    : serial_(0),
      pCam_(nullptr)  // Hack to suppress compiler warning.
                      // Spinnaker has only one contructor which takes an int
      ,
      camera_(nullptr),
      captureRunning_(false) {
  system_ = Spinnaker::System::GetInstance();

  const Spinnaker::LibraryVersion spinnakerLibraryVersion = system_->GetLibraryVersion();
  ROS_INFO_STREAM("Spinnaker library version: "
                  << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor << "."
                  << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build);

  ROS_INFO_STREAM("Retreiving list of cameras...");
  camList_ = system_->GetCameras();
  unsigned int num_cameras = camList_.GetSize();
  ROS_INFO_STREAM_ONCE("[SpinnakerCamera]: Number of cameras detected: " << num_cameras);

  // Setting pixel format bit length
  bit_set_16.insert("BayerGR16");
  bit_set_16.insert("BayerRG16");
  bit_set_16.insert("BayerGB16");
  bit_set_16.insert("BayerBG16");
  bit_set_16.insert("Mono16");

  bit_set_8.insert("BayerGR8");
  bit_set_8.insert("BayerRG8");
  bit_set_8.insert("BayerGB8");
  bit_set_8.insert("BayerBG8");
  bit_set_8.insert("Mono8");

  bit_set_24.insert("BGR8");
  bit_set_24.insert("RGB8Packed");
}

SpinnakerCamera::~SpinnakerCamera() {
  camList_.Clear();
  system_->ReleaseInstance();
}

void SpinnakerCamera::checkUSBMemory() {
  // Check USB Memory
  int mem;
  std::ifstream usb_mem("/sys/module/usbcore/parameters/usbfs_memory_mb");
  if (usb_mem) {
    usb_mem >> mem;
    if (mem >= 1000)
      ROS_INFO_STREAM("[ OK ] USB memory: " << mem << " MB");
    else {
      ROS_FATAL_STREAM("  USB memory on system too low ("
                       << mem
                       << " MB)! Must be at least 1000 MB. Run: \nsudo sh -c \"echo 1000 "
                          "> /sys/module/usbcore/parameters/usbfs_memory_mb\"\n "
                          "Terminating...");
      ros::shutdown();
    }
  } else {
    ROS_FATAL_STREAM("Could not check USB memory on system! Terminating...");
    ros::shutdown();
  }
}

void SpinnakerCamera::setNewConfiguration(const spinnaker_camera_driver::SpinnakerConfig& config,
                                          const uint32_t& level) {
  // Check if camera is connected
  if (!pCam_) {
    // Check USB memory only the first time
    SpinnakerCamera::checkUSBMemory();
    SpinnakerCamera::connect();
  }

  // Activate mutex to prevent us from grabbing images during this time
  std::lock_guard<std::mutex> scopedLock(mutex_);

  if (level >= LEVEL_RECONFIGURE_STOP) {
    ROS_DEBUG("SpinnakerCamera::setNewConfiguration: Reconfigure Stop.");
    bool capture_was_running = captureRunning_;
    start();  // For some reason some params only work after aquisition has be
              // started once.
    stop();
    camera_->setNewConfiguration(config, level);
    if (capture_was_running) start();
  } else {
    camera_->setNewConfiguration(config, level);
  }
}  // end setNewConfiguration

void SpinnakerCamera::setGain(const float& gain) {
  if (camera_) camera_->setGain(gain);
}

int SpinnakerCamera::getHeightMax() {
  if (camera_)
    return camera_->getHeightMax();
  else
    return 0;
}

int SpinnakerCamera::getWidthMax() {
  if (camera_)
    return camera_->getWidthMax();
  else
    return 0;
}

Spinnaker::GenApi::CNodePtr SpinnakerCamera::readProperty(
    const Spinnaker::GenICam::gcstring property_name) {
  if (camera_) {
    return camera_->readProperty(property_name);
  } else {
    return 0;
  }
}

void SpinnakerCamera::connect() {
  if (!pCam_) {
    // If we have a specific camera to connect to (specified by a serial number)
    if (serial_ != 0) {
      const auto serial_string = std::to_string(serial_);

      try {
        pCam_ = camList_.GetBySerial(serial_string);
      } catch (const Spinnaker::Exception& e) {
        throw std::runtime_error(
            "[SpinnakerCamera::connect] Could not find camera with serial "
            "number " +
            serial_string + ". Is that camera plugged in? Error: " + std::string(e.what()));
      }
    } else {
      // Connect to any camera (the first)
      try {
        pCam_ = camList_.GetByIndex(0);
      } catch (const Spinnaker::Exception& e) {
        throw std::runtime_error(
            "[SpinnakerCamera::connect] Failed to get first connected camera. "
            "Error: " +
            std::string(e.what()));
      }
    }
    if (!pCam_ || !pCam_->IsValid()) {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to obtain camera reference.");
    }

    try {
      // Check Device type and save serial for reconnecting
      Spinnaker::GenApi::INodeMap& genTLNodeMap = pCam_->GetTLDeviceNodeMap();

      if (serial_ == 0) {
        Spinnaker::GenApi::CStringPtr serial_ptr =
            static_cast<Spinnaker::GenApi::CStringPtr>(genTLNodeMap.GetNode("DeviceSerialNumber"));
        if (IsAvailable(serial_ptr) && IsReadable(serial_ptr)) {
          serial_ = atoi(serial_ptr->GetValue().c_str());
          ROS_INFO("[SpinnakerCamera::connect]: Using Serial: %i", serial_);
        } else {
          throw std::runtime_error(
              "[SpinnakerCamera::connect]: Unable to determine serial number.");
        }
      }

      Spinnaker::GenApi::CEnumerationPtr device_type_ptr =
          static_cast<Spinnaker::GenApi::CEnumerationPtr>(genTLNodeMap.GetNode("DeviceType"));

      if (IsAvailable(device_type_ptr) && IsReadable(device_type_ptr)) {
        ROS_INFO_STREAM(
            "[SpinnakerCamera::connect]: Detected device type: " << device_type_ptr->ToString());

        if (device_type_ptr->GetCurrentEntry() == device_type_ptr->GetEntryByName("U3V")) {
          Spinnaker::GenApi::CEnumerationPtr device_speed_ptr =
              static_cast<Spinnaker::GenApi::CEnumerationPtr>(
                  genTLNodeMap.GetNode("DeviceCurrentSpeed"));
          if (IsAvailable(device_speed_ptr) && IsReadable(device_speed_ptr)) {
            if (device_speed_ptr->GetCurrentEntry() !=
                device_speed_ptr->GetEntryByName("SuperSpeed"))
              ROS_ERROR_STREAM(
                  "[SpinnakerCamera::connect]: U3V Device not running at "
                  "Super-Speed. Check Cables! ");
          }
        }
        // TODO(mhosmar): - check if interface is GigE and connect to GigE cam
      }
    } catch (const Spinnaker::Exception& e) {
      throw std::runtime_error(
          "[SpinnakerCamera::connect] Failed to determine device info with "
          "error: " +
          std::string(e.what()));
    }

    try {
      // Initialize Camera
      pCam_->Init();

      // Retrieve GenICam nodemap
      node_map_ = &pCam_->GetNodeMap();

      // detect model and set camera_ accordingly;
      Spinnaker::GenApi::CStringPtr model_name = node_map_->GetNode("DeviceModelName");
      std::string model_name_str(model_name->ToString());

      ROS_INFO("[SpinnakerCamera::connect]: Camera model name: %s", model_name_str.c_str());
      if (is_bluerov_camera_)
        camera_.reset(new BlueROVCamera(node_map_));
      else if (model_name_str.find("Blackfly S") != std::string::npos)
        camera_.reset(new Camera(node_map_));
      else if (model_name_str.find("Chameleon3") != std::string::npos)
        camera_.reset(new Cm3(node_map_));
      else if (model_name_str.find("Grasshopper3") != std::string::npos)
        camera_.reset(new Gh3(node_map_));
      else {
        camera_.reset(new Camera(node_map_));
        ROS_WARN("SpinnakerCamera::connect: Could not detect camera model name.");
      }

      // Configure chunk data - Enable Metadata
      // SpinnakerCamera::ConfigureChunkData(*node_map_);
    } catch (const Spinnaker::Exception& e) {
      throw std::runtime_error("[SpinnakerCamera::connect] Failed to connect to camera. Error: " +
                               std::string(e.what()));
    } catch (const std::runtime_error& e) {
      throw std::runtime_error(
          "[SpinnakerCamera::connect] Failed to configure chunk data. Error: " +
          std::string(e.what()));
    }
  }

  // TODO(mhosmar): Get camera info to check if camera is running in color or
  // mono mode
  /*
  CameraInfo cInfo;
  error = cam_.GetCameraInfo(&cInfo);
  SpinnakerCamera::handleError("SpinnakerCamera::connect  Failed to get camera
  info.", error); isColor_ = cInfo.isColorCamera;
  */
}

void SpinnakerCamera::disconnect() {
  std::lock_guard<std::mutex> scopedLock(mutex_);
  captureRunning_ = false;
  try {
    // Check if camera is connected
    if (pCam_) {
      pCam_->DeInit();
      pCam_ = static_cast<int>(NULL);
      camList_.RemoveBySerial(std::to_string(serial_));
    }
    Spinnaker::CameraList temp_list = system_->GetCameras();
    camList_.Append(temp_list);
  } catch (const Spinnaker::Exception& e) {
    throw std::runtime_error(
        "[SpinnakerCamera::disconnect] Failed to disconnect camera with "
        "error: " +
        std::string(e.what()));
  }
}

void SpinnakerCamera::start() {
  try {
    // Check if camera is connected
    if (pCam_ && !captureRunning_) {
      // Start capturing images
      pCam_->BeginAcquisition();
      captureRunning_ = true;
    }
  } catch (const Spinnaker::Exception& e) {
    throw std::runtime_error("[SpinnakerCamera::start] Failed to start capture with error: " +
                             std::string(e.what()));
  }
}

void SpinnakerCamera::stop() {
  if (pCam_ && captureRunning_) {
    // Stop capturing images
    try {
      captureRunning_ = false;
      pCam_->EndAcquisition();
    } catch (const Spinnaker::Exception& e) {
      throw std::runtime_error("[SpinnakerCamera::stop] Failed to stop capture with error: " +
                               std::string(e.what()));
    }
  }
}

void SpinnakerCamera::grabImage(sensor_msgs::Image* image, const std::string& frame_id) {
  std::lock_guard<std::mutex> scopedLock(mutex_);

  // Check if Camera is connected and Running
  if (pCam_ && captureRunning_) {
    // Handle "Image Retrieval" Exception
    try {
      Spinnaker::ImagePtr image_ptr = pCam_->GetNextImage(timeout_);
      //  std::string format(image_ptr->GetPixelFormatName());
      //  std::printf("\033[100m format: %s \n", format.c_str());

      if (image_ptr->IsIncomplete()) {
        throw std::runtime_error("[SpinnakerCamera::grabImage] Image received from camera " +
                                 std::to_string(serial_) + " is incomplete.");
      } else {
        // Set Image Time Stamp
        uint64_t stamp = image_ptr->GetTimeStamp();
        image->header.stamp.sec = stamp * 1e-9;
        image->header.stamp.nsec = stamp % 1000000000;

        // Check the bits per pixel.
        size_t bitsPerPixel = image_ptr->GetBitsPerPixel();

        // --------------------------------------------------
        // Set the image encoding
        std::string pixel_format = camera_->getPixelFormat();
        uint16_t pixel_size = getbitsPerPixel(pixel_format);

        // ROS_DEBUG_STREAM("Pixel Format: " << pixel_format);
        ROS_FATAL_STREAM_COND(pixel_size != bitsPerPixel,
                              "Bits Per Pixel do not match between image and format."
                                  << bitsPerPixel << " vs " << pixel_size);
        std::string imageEncoding = getRosImageEncoding(pixel_format);
        // ROS_DEBUG_STREAM("Ros Image Encoding: " << imageEncoding);

        Spinnaker::GenApi::CEnumerationPtr color_filter_ptr =
            static_cast<Spinnaker::GenApi::CEnumerationPtr>(node_map_->GetNode("PixelColorFilter"));

        Spinnaker::GenICam::gcstring color_filter_str = color_filter_ptr->ToString();

        // ROS_DEBUG_STREAM("Pixel Color Filter: " << color_filter_str.c_str());
        int width = image_ptr->GetWidth();
        int height = image_ptr->GetHeight();
        int stride = image_ptr->GetStride();

        // ROS_INFO_ONCE("\033[93m wxh: (%d, %d), stride: %d \n", width, height,
        // stride);
        // std::string decoding_format = "PixelFormat_" + pixel_format;
        // ROS_DEBUG_STREAM_THROTTLE(60, "Deconding format: " <<
        // decoding_format);

        // Spinnaker::PixelFormatEnums decoding_format =
        //     getPixelFormatEnum(pixel_format);

        // try {
        //   Spinnaker::ImagePtr converted_image =
        //       image_ptr->Convert(decoding_format, Spinnaker::HQ_LINEAR);
        // } catch (const Spinnaker::Exception& e) {
        //   throw std::runtime_error("[SpinnakerCamera::grabImage]" +
        //                            std::string(e.what()));
        // }

        // ROS_DEBUG_STREAM_THROTTLE(
        //     30,
        //     "width: " << width << "height: " << height << "stride: " <<
        //     stride
        //               << "Xpadding: " << XPadding << "YPadding: " << YPadding
        //               << "Encoding: " << imageEncoding);

        fillImage(*image, imageEncoding, height, width, stride, image_ptr->GetData());
        image->header.frame_id = frame_id;
      }  // end else
    } catch (const Spinnaker::Exception& e) {
      throw std::runtime_error(
          "[SpinnakerCamera::grabImage] Failed to retrieve buffer with "
          "error: " +
          std::string(e.what()));
    }
  } else if (pCam_) {
    throw CameraNotRunningException(
        "[SpinnakerCamera::grabImage] Camera is currently not running.  Please "
        "start "
        "capturing frames first.");
  } else {
    throw std::runtime_error("[SpinnakerCamera::grabImage] Not connected to the camera.");
  }
}  // end grabImage

void SpinnakerCamera::setTimeout(const double& timeout) {
  timeout_ = static_cast<uint64_t>(std::round(timeout * 1000));
}
void SpinnakerCamera::setDesiredCamera(const uint32_t& id) { serial_ = id; }

void SpinnakerCamera::ConfigureChunkData(const Spinnaker::GenApi::INodeMap& nodeMap) {
  ROS_INFO_STREAM("*** CONFIGURING CHUNK DATA ***");
  try {
    // Activate chunk mode
    //
    // *** NOTES ***
    // Once enabled, chunk data will be available at the end of the payload
    // of every image captured until it is disabled. Chunk data can also be
    // retrieved from the nodemap.
    //
    Spinnaker::GenApi::CBooleanPtr ptrChunkModeActive = nodeMap.GetNode("ChunkModeActive");
    if (!Spinnaker::GenApi::IsAvailable(ptrChunkModeActive) ||
        !Spinnaker::GenApi::IsWritable(ptrChunkModeActive)) {
      throw std::runtime_error("Unable to activate chunk mode. Aborting...");
    }
    ptrChunkModeActive->SetValue(true);
    ROS_INFO_STREAM_ONCE("Chunk mode activated...");

    // Enable all types of chunk data
    //
    // *** NOTES ***
    // Enabling chunk data requires working with nodes: "ChunkSelector"
    // is an enumeration selector node and "ChunkEnable" is a boolean. It
    // requires retrieving the selector node (which is of enumeration node
    // type), selecting the entry of the chunk data to be enabled, retrieving
    // the corresponding boolean, and setting it to true.
    //
    // In this example, all chunk data is enabled, so these steps are
    // performed in a loop. Once this is complete, chunk mode still needs to
    // be activated.
    //
    Spinnaker::GenApi::NodeList_t entries;
    // Retrieve the selector node
    Spinnaker::GenApi::CEnumerationPtr ptrChunkSelector = nodeMap.GetNode("ChunkSelector");
    if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelector) ||
        !Spinnaker::GenApi::IsReadable(ptrChunkSelector)) {
      throw std::runtime_error("Unable to retrieve chunk selector. Aborting...");
    }
    // Retrieve entries
    ptrChunkSelector->GetEntries(entries);

    ROS_INFO_STREAM("Enabling entries...");

    for (unsigned int i = 0; i < entries.size(); i++) {
      // Select entry to be enabled
      Spinnaker::GenApi::CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);
      // Go to next node if problem occurs
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelectorEntry) ||
          !Spinnaker::GenApi::IsReadable(ptrChunkSelectorEntry)) {
        continue;
      }
      ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

      ROS_INFO_STREAM("\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ");
      // Retrieve corresponding boolean
      Spinnaker::GenApi::CBooleanPtr ptrChunkEnable = nodeMap.GetNode("ChunkEnable");
      // Enable the boolean, thus enabling the corresponding chunk data
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkEnable)) {
        ROS_INFO("Node not available");
      } else if (ptrChunkEnable->GetValue()) {
        ROS_INFO("Enabled");
      } else if (Spinnaker::GenApi::IsWritable(ptrChunkEnable)) {
        ptrChunkEnable->SetValue(true);
        ROS_INFO("Enabled");
      } else {
        ROS_INFO("Node not writable");
      }
    }
  } catch (const Spinnaker::Exception& e) {
    throw std::runtime_error(e.what());
  }
}

void SpinnakerCamera::setBlueROVCamera(const bool& bluerov_camera) {
  is_bluerov_camera_ = bluerov_camera;
}

uint16_t SpinnakerCamera::getbitsPerPixel(const std::string& image_format) {
  if (bit_set_16.find(image_format) != bit_set_16.end())
    return 16;
  else if (bit_set_24.find(image_format) != bit_set_24.end())
    return 24;
  else if (bit_set_8.find(image_format) != bit_set_8.end())
    return 8;
  else
    return 0;
}

std::string SpinnakerCamera::getRosImageEncoding(const std::string& image_format) {
  if (image_format == "BayerRG8") {
    return sensor_msgs::image_encodings::BAYER_RGGB8;
  } else if (image_format == "BayerGR8") {
    return sensor_msgs::image_encodings::BAYER_GRBG8;
  } else if (image_format == "BayerGB8") {
    return sensor_msgs::image_encodings::BAYER_GBRG8;
  } else if (image_format == "BayerBG8") {
    return sensor_msgs::image_encodings::BAYER_BGGR8;
  } else if (image_format == "BayerRG16") {
    return sensor_msgs::image_encodings::BAYER_RGGB16;
  } else if (image_format == "BayerGR16") {
    return sensor_msgs::image_encodings::BAYER_GRBG16;
  } else if (image_format == "BayerGB16") {
    return sensor_msgs::image_encodings::BAYER_GBRG16;
  } else if (image_format == "BayerBG16") {
    return sensor_msgs::image_encodings::BAYER_BGGR16;
  } else if (image_format == "Mono8") {
    return sensor_msgs::image_encodings::MONO8;
  } else if (image_format == "Mono16") {
    return sensor_msgs::image_encodings::MONO16;
  } else if (image_format == "RGB8Packed") {
    return sensor_msgs::image_encodings::RGB8;
  } else if (image_format == "BGR8") {
    return sensor_msgs::image_encodings::BGR8;
  } else {
    return "";
  }
}

Spinnaker::PixelFormatEnums SpinnakerCamera::getPixelFormatEnum(const std::string& image_format) {
  if (image_format == "BayerRG8") {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerRG8;
  } else if (image_format == "BayerGR8") {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerGR8;
  } else if (image_format == "BayerGB8") {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerGB8;
  } else if (image_format == "BayerBG8") {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerBG8;
  } else if (image_format == "BayerRG16") {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerRG16;
  } else if (image_format == "BayerGR16") {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerGR16;
  } else if (image_format == "BayerGB16") {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerGB16;
  } else if (image_format == "BayerBG16") {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerBG16;
  } else if (image_format == "Mono8") {
    return Spinnaker::PixelFormatEnums::PixelFormat_Mono8;
  } else if (image_format == "Mono16") {
    return Spinnaker::PixelFormatEnums::PixelFormat_Mono16;
  } else if (image_format == "RGB8Packed") {
    return Spinnaker::PixelFormatEnums::PixelFormat_RGB8Packed;
  } else if (image_format == "BGR8") {
    return Spinnaker::PixelFormatEnums::PixelFormat_BGR8;
  }
}

}  // namespace spinnaker_camera_driver
