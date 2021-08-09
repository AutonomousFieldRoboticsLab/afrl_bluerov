/**
Software License Agreement (BSD)

\file      set_property.h
\authors   Bharat Joshi <bjoshi@email.sc.edu>
\copyright Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.

*/
#pragma once

#include <string>

#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

namespace spinnaker_camera_driver {

inline bool getFloatValueMax(Spinnaker::GenApi::INodeMap* node_map,
                             const std::string& property_name,
                             double& max_value) {
  Spinnaker::GenApi::CFloatPtr floatPtr = node_map->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(floatPtr)) {
    ROS_ERROR_STREAM("[SpinnakerCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(
                                                   node_map->GetNode("DeviceSerialNumber"))
                                                   ->GetValue()
                                            << ") Feature name " << property_name
                                            << " not implemented.");
    return false;
  }

  if (Spinnaker::GenApi::IsAvailable(floatPtr)) {
    max_value = floatPtr->GetMax();
    return true;
  } else {
    ROS_WARN_STREAM("[SpinnakerCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(
                                                  node_map->GetNode("DeviceSerialNumber"))
                                                  ->GetValue()
                                           << ") Feature " << property_name << " not available.");
  }
  return false;
}

inline bool getIntValueMax(Spinnaker::GenApi::INodeMap* node_map,
                           const std::string& property_name,
                           int& max_value) {
  Spinnaker::GenApi::CIntegerPtr intPtr = node_map->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(intPtr)) {
    ROS_ERROR_STREAM("[SpinnakerCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(
                                                   node_map->GetNode("DeviceSerialNumber"))
                                                   ->GetValue()
                                            << ") Feature name " << property_name
                                            << " not implemented.");
    return false;
  }

  if (Spinnaker::GenApi::IsAvailable(intPtr)) {
    max_value = intPtr->GetMax();
    return true;
  } else {
    ROS_WARN_STREAM("[SpinnakerCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(
                                                  node_map->GetNode("DeviceSerialNumber"))
                                                  ->GetValue()
                                           << ") Feature " << property_name << " not available.");
  }
  return false;
}

inline bool getFloatValueMin(Spinnaker::GenApi::INodeMap* node_map,
                             const std::string& property_name,
                             double& max_value) {
  Spinnaker::GenApi::CFloatPtr floatPtr = node_map->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(floatPtr)) {
    ROS_ERROR_STREAM("[SpinnakerCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(
                                                   node_map->GetNode("DeviceSerialNumber"))
                                                   ->GetValue()
                                            << ") Feature name " << property_name
                                            << " not implemented.");
    return false;
  }

  if (Spinnaker::GenApi::IsAvailable(floatPtr)) {
    max_value = floatPtr->GetMin();
    return true;
  } else {
    ROS_WARN_STREAM("[SpinnakerCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(
                                                  node_map->GetNode("DeviceSerialNumber"))
                                                  ->GetValue()
                                           << ") Feature " << property_name << " not available.");
  }
  return false;
}

inline bool getIntValueMin(Spinnaker::GenApi::INodeMap* node_map,
                           const std::string& property_name,
                           int& max_value) {
  Spinnaker::GenApi::CIntegerPtr intPtr = node_map->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(intPtr)) {
    ROS_ERROR_STREAM("[SpinnakerCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(
                                                   node_map->GetNode("DeviceSerialNumber"))
                                                   ->GetValue()
                                            << ") Feature name " << property_name
                                            << " not implemented.");
    return false;
  }

  if (Spinnaker::GenApi::IsAvailable(intPtr)) {
    max_value = intPtr->GetMin();
    return true;
  } else {
    ROS_WARN_STREAM("[SpinnakerCamera]: (" << static_cast<Spinnaker::GenApi::CStringPtr>(
                                                  node_map->GetNode("DeviceSerialNumber"))
                                                  ->GetValue()
                                           << ") Feature " << property_name << " not available.");
  }
  return false;
}

}  // namespace spinnaker_camera_driver
