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

#ifndef ROS2_OUSTER__PREPROCESSORS__LIDARSCAN_PREPROCESSOR_HPP_
#define ROS2_OUSTER__PREPROCESSORS__LIDARSCAN_PREPROCESSOR_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/data_preprocessor_interface.hpp"

namespace viz = ouster::viz;

namespace sensor
{
/**
 * @class sensor::LidarscanPreprocessor
 * @brief The LidarscanPreprocessor creates LidarScan for the processors from
 * packet data.
 */
class LidarscanPreprocessor
  : public ros2_ouster::DataPreProcessorInterface<ouster::LidarScan>
{
public:
  LidarscanPreprocessor(
    const ouster::sensor::sensor_info & mdata,
    const ouster::sensor::packet_format & pf)
  : _pf(pf)
  {
    _batch = std::make_unique<ouster::ScanBatcher>(mdata.format.columns_per_frame, _pf);
    _ls = std::make_shared<ouster::LidarScan>(
      ouster::LidarScan{mdata.format.columns_per_frame,
        mdata.format.pixels_per_column});
  }

  /**
   * @brief Returns true is data is ready also activates the preprocessor
   */
  bool isDataReady() override
  {
    _activated = true;
    return _dataReady;
  }

  /**
   * @brief Returns the ready lidarscan. If the lidarscan is not ready it will
   * throw an exception
   */
  std::shared_ptr<ouster::LidarScan> getData() override
  {
    if (!_dataReady) {
      throw ros2_ouster::OusterDriverException("Preprocessor data not ready.");
    }

    return _ls;
  }

  /**
   * @brief Returns the ready timestamp. If the timestamp is not ready it will
   * throw an exception
   */
  ts_t getTimestamp() override
  {
    if (!_dataReady) {
      throw ros2_ouster::OusterDriverException("Preprocessor data not ready.");
    }

    return _timestamp;
  }

  /**
   * @brief Takes packet data to preprocess it into a lidarscan
   */
  bool preprocess(const uint8_t * data, uint64_t override_ts) override
  {
    if (!_activated) {
      return true;
    }

    if (_dataReady) {
      _dataReady = false;
    }

    handle(data, override_ts);
    return true;
  }

private:
  /**
  * @brief Private function handling the packet data and batching it into a
   * lidarscan
  */
  bool handle(const uint8_t * data, uint64_t override_ts) override
  {
    if (_batch->operator()(data, *_ls)) {
      auto h = std::find_if(
        _ls->headers.begin(), _ls->headers.end(), [](const auto & h) {
          return h.timestamp != std::chrono::nanoseconds{0};
        });
      if (h != _ls->headers.end()) {
        _timestamp = h->timestamp;
      }
      _dataReady = true;
    }
    return true;
  }

  bool _dataReady;
  bool _activated;
  ts_t _timestamp;
  std::unique_ptr<ouster::ScanBatcher> _batch;
  std::shared_ptr<ouster::LidarScan> _ls;
  ouster::sensor::packet_format _pf;
};

}  // namespace sensor

#endif  // ROS2_OUSTER__PREPROCESSORS__LIDARSCAN_PREPROCESSOR_HPP_
