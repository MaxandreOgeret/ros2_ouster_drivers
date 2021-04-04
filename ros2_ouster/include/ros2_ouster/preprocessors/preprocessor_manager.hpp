// Copyright 2021, Steve Macenski
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_OUSTER__PREPROCESSOR_MANAGER_HPP_
#define ROS2_OUSTER__PREPROCESSOR_MANAGER_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"

#include "ros2_ouster/conversions.hpp"
#include "ros2_ouster/exception.hpp"

#include "ros2_ouster/client/client.h"
#include "ros2_ouster/client/viz/autoexposure.h"
#include "ros2_ouster/client/viz/beam_uniformity.h"
#include "ros2_ouster/preprocessors/lidarscan_preprocessor.hpp"

namespace viz = ouster::viz;

namespace sensor
{

/**
 * @class sensor::PreprocessorManager
 * @brief This class manages the preprocessors. It will create the preprocessors
 * It is also passed to the processors so thery can access the preprocessors
 */
class PreprocessorManager
{
public:
  /**
  * @brief Constructor of the PreprocessorManager
  */
  PreprocessorManager(
    const ouster::sensor::sensor_info & mdata,
    const ouster::sensor::packet_format & pf)
  {
    _ls_preprocessor = std::make_shared<LidarscanPreprocessor>(mdata, pf);
  }

  /**
  * @brief Passes the packet data to the preprocessors
  */
  void preprocess(const uint8_t * data, uint64_t override_ts)
  {
    _ls_preprocessor->preprocess(data, override_ts);
  }

  /**
  * @brief Returns the lidarscan preprocessor
  */
  std::shared_ptr<sensor::LidarscanPreprocessor> get_lidarscan_preprocessor()
  {
    return _ls_preprocessor;
  }

private:
  std::shared_ptr<sensor::LidarscanPreprocessor> _ls_preprocessor;

};

}  // namespace sensor

#endif  // ROS2_OUSTER__PREPROCESSOR_MANAGER_HPP_
