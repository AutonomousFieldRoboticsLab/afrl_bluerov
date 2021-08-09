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
#pragma once

#include "spinnaker_camera_driver/camera.h"

namespace spinnaker_camera_driver {
class BlueROVCamera : public Camera {
 public:
  explicit BlueROVCamera(Spinnaker::GenApi::INodeMap* node_map);
  ~BlueROVCamera();

  void setNewConfiguration(const SpinnakerConfig& config, const uint32_t& level) override;

  void init() override;

  void setImageControlFormats(const spinnaker_camera_driver::SpinnakerConfig& config) override;

  bool isConfigChanged(const std::string& config_name,
                       const spinnaker_camera_driver::SpinnakerConfig& new_config);

 private:
  SpinnakerConfig old_config_;

  const std::string binning_selector_ = "All";
  const std::string binning_mode_ = "Average";
  const std::string blue = "Blue";
  const std::string red = "Red";

  bool static_configs_set_;
  double frame_rate_;
};

}  // namespace spinnaker_camera_driver
