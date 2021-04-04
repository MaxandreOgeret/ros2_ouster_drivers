#ifndef ROS2_OUSTER__INTERFACES__DATA_PREPROCESSOR_INTERFACE_HPP_
#define ROS2_OUSTER__INTERFACES__DATA_PREPROCESSOR_INTERFACE_HPP_

#include <memory>
#include <string>

using ts_t = std::chrono::nanoseconds;

namespace ros2_ouster
{
/**
 * @class ros2_ouster::DataPreProcessorInterface
 * @brief An interface for data preprocessors using a
 * lidar-specific API to preprocess data for the processors
 * It has an access point for the processor.
 */
template<class T>
class DataPreProcessorInterface
{
public:
  /**
   * @brief Constructor of the data processor interface
   */
  DataPreProcessorInterface() {}

  /**
   * @brief Destructor of the data processor interface
   */
  virtual ~DataPreProcessorInterface() = default;

private:
  /**
   * @brief Returns true is data is ready also activates the preprocessor
   */
  virtual bool isDataReady() = 0;

  /**
   * @brief Returns the preprocessed data of type T
   */
  virtual std::shared_ptr<T> getData() = 0;

  /**
   * @brief Returns the preprocessed data of type T
   */
  virtual ts_t getTimestamp() = 0;

  /**
   * @brief Process a packet with the lidar-specific APIs. Public accessor.
   * @param data packet input
   * @param override_ts Timestamp in nanos to use to override the ts in the
   *                    packet data. To use the packet data, pass as 0.
   */
  virtual bool preprocess(const uint8_t * data, uint64_t override_ts) = 0;

  /**
   * @brief Handles the data passed to preprocess. Private.
   * @param data packet input
   * @param override_ts Timestamp in nanos to use to override the ts in the
   *                    packet data. To use the packet data, pass as 0.
   */
  virtual bool handle(const uint8_t * data, uint64_t override_ts) = 0;


};

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__INTERFACES__DATA_PREPROCESSOR_INTERFACE_HPP_
