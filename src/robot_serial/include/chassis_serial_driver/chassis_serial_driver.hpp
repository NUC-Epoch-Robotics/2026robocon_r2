#ifndef CHASSIS_SERIAL_DRIVER__CHASSIS_SERIAL_DRIVER_HPP_
#define CHASSIS_SERIAL_DRIVER__CHASSIS_SERIAL_DRIVER_HPP_


#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "robot_serial/msg/command.hpp"
#include "robot_serial/msg/ack.hpp"
  
namespace chassis_serial_driver
{
class ChassisSerialDriver : public rclcpp::Node
{
public:
  explicit ChassisSerialDriver(const rclcpp::NodeOptions & options);

  ~ChassisSerialDriver() override;

private:
  void getParams();

  void receiveData0();

  void sendData();
  
  void getData1(const robot_serial::msg::Command::SharedPtr cmd);

  void reopenPort();
 
  void get_location(const nav_msgs::msg::Odometry::SharedPtr location);
  // void setParam(const rclcpp::Parameter & param);

  // Serial port
  
  int serial_count;
  int Context_count;
  float cmd_x_=0.0f;
  float cmd_y_=0.0f;
  float cmd_yaw_=0.0000f;
  float cmd_x_1=0.0f;
  float cmd_y_1=0.0f;
  float cmd_yaw_1=0.0000f;
  uint8_t spearhead_=0;
  uint8_t block_=0;
  uint8_t stair_=0;
  uint8_t area_=0;
  uint8_t dt35_=0;
  std::vector<std::string> device_name_;
  std::vector<std::unique_ptr<IoContext>> owned_ctx_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::vector<std::unique_ptr<drivers::serial_driver::SerialDriver>> serial_driver_;



  // Broadcast tf from odom to gimbal_link
  // double timestamp_offset_ = 0;
  // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<robot_serial::msg::Command>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<robot_serial::msg::Ack>::SharedPtr ack_pub_;
  rclcpp::Publisher<robot_serial::msg::Command>::SharedPtr dt35_pub_;
 
 
  // For debug usage
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
 
  std::thread receive_thread_0;
  std::thread receive_thread_;
};
}  // namespace chassis_serial_driver

#endif  // Chassis_SERIAL_DRIVER__Chassis_SERIAL_DRIVER_HPP_
