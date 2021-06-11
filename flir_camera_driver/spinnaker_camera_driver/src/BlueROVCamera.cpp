/**
Software License Agreement (BSD)

\file      BlueROVCamera.h
\authors   Michael Hosmar <mhosmar@clearpathrobotics.com>
\authors   Bharat Joshi <bjoshi@email.sc.edu>
\copyright Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.
\copyright Copyright (c) 2021, University of South Carolina., All rights
reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WAR- RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, IN- DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "spinnaker_camera_driver/BlueROVCamera.h"

namespace spinnaker_camera_driver {

BlueROVCamera::BlueROVCamera(Spinnaker::GenApi::INodeMap* node_map)
    : Camera(node_map) {
  init();
  pixel_format_ = "Mono8";
}

BlueROVCamera::~BlueROVCamera() {}

void BlueROVCamera::setNewConfiguration(const SpinnakerConfig& config,
                                        const uint32_t& level) {
  try {
    if (level >= LEVEL_RECONFIGURE_STOP) setImageControlFormats(config);

    setFrameRate(static_cast<float>(config.acquisition_frame_rate));
    // Set enable after frame rate encase its false
    setProperty(node_map_,
                "AcquisitionFrameRateEnable",
                config.acquisition_frame_rate_enable);

    // Set Trigger and Strobe Settings
    // NOTE: The trigger must be disabled (i.e. TriggerMode = "Off") in order to
    // configure whether the source is software or hardware.
    setProperty(node_map_, "TriggerMode", std::string("Off"));
    setProperty(node_map_, "TriggerSource", config.trigger_source);
    setProperty(node_map_, "TriggerSelector", config.trigger_selector);
    setProperty(node_map_, "TriggerActivation", config.trigger_activation_mode);
    setProperty(node_map_, "TriggerMode", config.enable_trigger);

    setProperty(node_map_, "LineSelector", config.line_selector);
    setProperty(node_map_, "LineMode", config.line_mode);
    setProperty(node_map_, "LineSource", config.line_source);

    // Set auto exposure
    setProperty(node_map_, "ExposureMode", config.exposure_mode);
    setProperty(node_map_, "ExposureAuto", config.exposure_auto);

    // Set sharpness
    if (IsAvailable(node_map_->GetNode("SharpeningEnable"))) {
      setProperty(node_map_, "SharpeningEnable", config.sharpening_enable);
      if (config.sharpening_enable) {
        setProperty(node_map_, "SharpeningAuto", config.auto_sharpness);
        setProperty(
            node_map_, "Sharpening", static_cast<float>(config.sharpness));
        setProperty(node_map_,
                    "SharpeningThreshold",
                    static_cast<float>(config.sharpening_threshold));
      }
    }

    // Set saturation
    if (IsAvailable(node_map_->GetNode("SaturationEnable"))) {
      setProperty(node_map_, "SaturationEnable", config.saturation_enable);
      if (config.saturation_enable) {
        setProperty(
            node_map_, "Saturation", static_cast<float>(config.saturation));
      }
    }

    // Set shutter time/speed
    if (config.exposure_auto.compare(std::string("Off")) == 0) {
      setProperty(
          node_map_, "ExposureTime", static_cast<float>(config.exposure_time));
    } else {
      setProperty(node_map_,
                  "AutoExposureExposureTimeUpperLimit",
                  static_cast<float>(config.auto_exposure_time_upper_limit));
    }

    // Set gain
    setProperty(node_map_, "GainSelector", config.gain_selector);
    setProperty(node_map_, "GainAuto", config.auto_gain);
    if (config.auto_gain.compare(std::string("Off")) == 0) {
      setProperty(node_map_, "Gain", static_cast<float>(config.gain));
    }

    // Set brightness
    setProperty(node_map_, "BlackLevel", static_cast<float>(config.brightness));

    // Set gamma
    if (config.gamma_enable) {
      setProperty(node_map_, "GammaEnable", config.gamma_enable);
      setProperty(node_map_, "Gamma", static_cast<float>(config.gamma));
    }

    // Set white balance
    if (IsAvailable(node_map_->GetNode("BalanceWhiteAuto"))) {
      setProperty(node_map_, "BalanceWhiteAuto", config.auto_white_balance);
      if (config.auto_white_balance.compare(std::string("Off")) == 0) {
        setProperty(node_map_, "BalanceRatioSelector", "Blue");
        setProperty(node_map_,
                    "BalanceRatio",
                    static_cast<float>(config.white_balance_blue_ratio));
        setProperty(node_map_, "BalanceRatioSelector", "Red");
        setProperty(node_map_,
                    "BalanceRatio",
                    static_cast<float>(config.white_balance_red_ratio));
      }
    }

    // Set Auto exposure/white balance parameters
    if (IsAvailable(node_map_->GetNode("AutoAlgorithmSelector"))) {
      setProperty(node_map_, "AutoAlgorithmSelector", std::string("Ae"));
      setProperty(node_map_, "AasRoiEnable", true);
      if (config.auto_exposure_roi_width != 0 &&
          config.auto_exposure_roi_height != 0) {
        setProperty(
            node_map_, "AasRoiOffsetX", config.auto_exposure_roi_offset_x);
        setProperty(
            node_map_, "AasRoiOffsetY", config.auto_exposure_roi_offset_y);
        setProperty(node_map_, "AasRoiWidth", config.auto_exposure_roi_width);
        setProperty(node_map_, "AasRoiHeight", config.auto_exposure_roi_height);
      }
    }

    // Set Auto exposure lighting mode
    if (IsAvailable(node_map_->GetNode("AutoExposureLightingMode"))) {
      setProperty(node_map_,
                  "AutoExposureLightingMode",
                  config.auto_exposure_lighting_mode);
    }
  } catch (const Spinnaker::Exception& e) {
    throw std::runtime_error(
        "[Camera::setNewConfiguration] Failed to set configuration: " +
        std::string(e.what()));
  }

  old_config_ = config;
}

/**
 * @brief Sets sensor height, width and throughput during initialization.
 * Oiginal implementation was using HeightMax and WidthMax property. But these
 * attributes are calculated after binning, so do not actaully the maxmimum
 * image size.
 */

void BlueROVCamera::init() {
  // Modified by Bharat

  Spinnaker::GenApi::CIntegerPtr sensor_height_ptr =
      node_map_->GetNode("SensorHeight");
  if (!IsAvailable(sensor_height_ptr) || !IsReadable(sensor_height_ptr)) {
    throw std::runtime_error("[Camera::init] Unable to read SensorHeight");
  }
  height_max_ = sensor_height_ptr->GetValue();
  Spinnaker::GenApi::CIntegerPtr sensor_width_ptr =
      node_map_->GetNode("SensorWidth");
  if (!IsAvailable(sensor_width_ptr) || !IsReadable(sensor_width_ptr)) {
    throw std::runtime_error("[Camera::init] Unable to read SensorWidth");
  }
  width_max_ = sensor_width_ptr->GetValue();
}

// Image Size and Pixel Format
void BlueROVCamera::setImageControlFormats(
    const spinnaker_camera_driver::SpinnakerConfig& config) {
  // Set Binning, and Reverse
  if (isConfigChanged("Binning", config)) {
    setProperty(node_map_, "BinningHorizontal", config.image_format_binning);
    setProperty(node_map_, "BinningVertical", config.image_format_binning);
    setProperty(node_map_, "BinningSelector", binning_selector_);
    setProperty(node_map_, "BinningHorizontalMode", binning_mode_);
    setProperty(node_map_, "BinningVerticalMode", binning_mode_);
  }

  if (isConfigChanged("ReverseX", config))
    setProperty(node_map_, "ReverseX", config.image_format_x_reverse);
  if (isConfigChanged("ReverseY", config))
    setProperty(node_map_, "ReverseY", config.image_format_y_reverse);

  // Grab the Max values after decimation
  Spinnaker::GenApi::CIntegerPtr height_max_ptr =
      node_map_->GetNode("HeightMax");
  if (!IsAvailable(height_max_ptr) || !IsReadable(height_max_ptr)) {
    throw std::runtime_error(
        "[Camera::setImageControlFormats] Unable to read HeightMax");
  }
  int height_max = height_max_ptr->GetValue();
  Spinnaker::GenApi::CIntegerPtr width_max_ptr = node_map_->GetNode("WidthMax");
  if (!IsAvailable(width_max_ptr) || !IsReadable(width_max_ptr)) {
    throw std::runtime_error(
        "[Camera::setImageControlFormats] Unable to read WidthMax");
  }
  int width_max = width_max_ptr->GetValue();

  // Offset first encase expanding ROI
  // Apply offset X
  //   setProperty(node_map_, "OffsetX", 0);
  // Apply offset Y
  //   setProperty(node_map_, "OffsetY", 0);

  // Set Width/Height
  //   if (config.image_format_roi_width <= 0 ||
  //   config.image_format_roi_width > width_max_)
  if (width_max_ != width_max) {
    width_max_ = width_max;
    setProperty(node_map_, "Width", width_max_);
  }
  //   else
  // setProperty(node_map_, "Width", config.image_format_roi_width);
  //   if (config.image_format_roi_height <= 0 ||
  //   config.image_format_roi_height > height_max_)
  if (height_max_ != height_max) {
    height_max_ = height_max;
    setProperty(node_map_, "Height", height_max_);
  }
  //   else
  // setProperty(node_map_, "Height", config.image_format_roi_height);

  // Apply offset X
  //   setProperty(node_map_, "OffsetX", config.image_format_x_offset);
  // Apply offset Y
  //   setProperty(node_map_, "OffsetY", config.image_format_y_offset);

  // Set Pixel Format
  if (isConfigChanged("PixelFormat", config))
    setProperty(node_map_, "PixelFormat", config.image_format_color_coding);
}

bool BlueROVCamera::isConfigChanged(
    const std::string& config_name,
    const spinnaker_camera_driver::SpinnakerConfig& new_config) {
  if (config_name == "Binning" &&
      old_config_.image_format_binning != new_config.image_format_binning) {
    ROS_DEBUG_STREAM("Binning changed from: "
                     << old_config_.image_format_binning << " to: "
                     << new_config.image_format_binning << std::endl);

    return true;
  } else if (config_name == "ReverseX" &&
             old_config_.image_format_x_reverse !=
                 new_config.image_format_x_reverse) {
    if (new_config.image_format_x_reverse)
      ROS_DEBUG_STREAM("Image is reversed in X direction\n");
    else
      ROS_DEBUG_STREAM("Image reverse corrected in X direction\n");
    return true;
  } else if (config_name == "ReverseY" &&
             old_config_.image_format_y_reverse !=
                 new_config.image_format_y_reverse) {
    if (new_config.image_format_y_reverse)
      ROS_DEBUG_STREAM("Image is reversed in Y direction\n");
    else
      ROS_DEBUG_STREAM("Image reverse corrected in Y direction\n");
    return true;
  } else if (config_name == "PixelFormat" &&
             pixel_format_ != new_config.image_format_color_coding) {
    ROS_DEBUG_STREAM("Image color encoding changed from: "
                     << pixel_format_
                     << " to: " << new_config.image_format_color_coding);
    return true;
  }
  return false;
}
}  // namespace spinnaker_camera_driver