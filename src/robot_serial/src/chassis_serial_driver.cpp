#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <cmath>    // [新增] 用于 std::abs 浮点数绝对值计算
#include <thread>   // [新增] 用于线程延时控制
#include <chrono>   // [新增] 用于时间单位处理

#include "chassis_serial_driver/crc.hpp"
#include "chassis_serial_driver/packet.hpp"
#include "chassis_serial_driver/chassis_serial_driver.hpp"
#include "robot_serial/msg/command.hpp"
#include "robot_serial/msg/ack.hpp"
namespace chassis_serial_driver
{
ChassisSerialDriver::ChassisSerialDriver(const rclcpp::NodeOptions & options)
: Node("chassis_serial_driver", options)
{
  RCLCPP_INFO(get_logger(), "Start ChassisSerialDriver!");
  RCLCPP_INFO(get_logger(), "Test,ok");
  serial_count =this->declare_parameter<int>("serial_count", 1);
  Context_count =this->declare_parameter<int>("Context_count", 1);

  // Initialize vectors with proper size
  device_name_.resize(serial_count);
  owned_ctx_.resize(Context_count);
  serial_driver_.resize(serial_count);

  for(int i=0;i<serial_count;i++){
  owned_ctx_[i] = std::make_unique<IoContext>(2);
  serial_driver_[i] = std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_[i]);
  };
  getParams();

  // ── 先创建 publisher，再启动接收线程，避免空指针 ──
  cmd_sub_ = this->create_subscription<robot_serial::msg::Command>(
    "command", rclcpp::SensorDataQoS(),
    std::bind(&ChassisSerialDriver::getData1, this, std::placeholders::_1));

  ack_pub_ = this->create_publisher<robot_serial::msg::Ack>("/juece_ack", rclcpp::SensorDataQoS());
  dt35_pub_ = this->create_publisher<robot_serial::msg::Command>("/dt35/location", rclcpp::SensorDataQoS());
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odin1/odometry",rclcpp::SensorDataQoS(),std::bind(&ChassisSerialDriver::get_location, this, std::placeholders::_1));

  // ── publisher 已就绪，再打开串口、启动接收线程 ──
  try {
    for(int i=0;i<serial_count;i++){
    serial_driver_[i]->init_port(device_name_[i], *device_config_);
    if (!serial_driver_[i]->port()->is_open()) {
      serial_driver_[i]->port()->open();
    }
  }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_[0].c_str(), ex.what());
    throw ex;
  }

  // 串口已打开，启动接收线程
  receive_thread_0 = std::thread(&ChassisSerialDriver::receiveData0, this);

  // ==================== 【关键修复区：初始状态发送】 ====================
  // 1. 硬件缓冲期：串口刚打开时，强行让线程睡 200 毫秒，防止电平不稳导致下位机漏读字节（吞包）
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // 2. 内存零初始化：加上 {} 括号，瞬间清空结构体在栈底的所有脏数据（包括 padding 和校验位）
  SendPacket packet{};

  // 3. 显式声明单精度：加上 'f'
  packet.x = 0.0f;
  packet.y = 0.0f;
  packet.yaw = 0.0000f;
  packet.spearhead = 0;
  packet.block = 0;
  packet.stair = 0;
  packet.area = 0;
  // 生成校验并发送
  //crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  std::vector<uint8_t> data = toVector(packet);
  serial_driver_[0]->port()->send(data);
}

ChassisSerialDriver::~ChassisSerialDriver()
{
  if (receive_thread_0.joinable()) {
    receive_thread_0.join();
  }
  for (size_t i = 0; i < static_cast<size_t>(serial_count) && i < serial_driver_.size(); i++) {
    if (serial_driver_[i] && serial_driver_[i]->port()->is_open()) {
      serial_driver_[i]->port()->close();
    }
  }

  for (size_t i = 0; i < static_cast<size_t>(Context_count) && i < owned_ctx_.size(); i++) {
    if (owned_ctx_[i]) {
      owned_ctx_[i]->waitForExit();
    }
  }
}

/**
 * @brief 接收下位机发来的数据
*/

void ChassisSerialDriver::receiveData0()
{
  if (serial_driver_.empty() || !serial_driver_[0]) {
    RCLCPP_WARN(get_logger(), "serial_driver_[0] not available");
    return;
  }

  std::vector<uint8_t> single_byte(1);
  std::vector<uint8_t> data;

  // 因为你加了 packed 属性，这里的 sizeof 绝对精准
  const size_t PAYLOAD_SIZE_1 = sizeof(ReceivePacket_1) - 2; // 3个字节
  const size_t PAYLOAD_SIZE_2 = sizeof(ReceivePacket) - 2;   // 8个字节 (2个float)

  while (rclcpp::ok()) {
    try {
      // 1. 寻找共同的第一帧头 0xAA
      serial_driver_[0]->port()->receive(single_byte);
      if (single_byte[0] != 0xAA) {
        continue;
      }

      // 2. 找到了 0xAA，紧接着读1个字节，根据它来进行"协议分流"
      serial_driver_[0]->port()->receive(single_byte);

      if (single_byte[0] == 0x55) {
        // ================== 处理第一类数据包：0xAA 0x55 ==================
        data.resize(PAYLOAD_SIZE_1);
        serial_driver_[0]->port()->receive(data);

        // 补回帧头，喂给你原有的 fromVector_1 解析函数
        data.insert(data.begin(), {0xAA, 0x55});
        ReceivePacket_1 packet1 = fromVector_1(data);
        
        // 发布 ROS 消息
        const robot_serial::msg::Ack::SharedPtr ack_msg = std::make_shared<robot_serial::msg::Ack>();
        ack_msg->xipan_status = packet1.xipan_zhuangtai;
        ack_msg->taijie_status = packet1.taijie_zhuangtai;
        ack_msg->up_free = packet1.up_free;
        ack_msg->down_free = packet1.down_free;
        ack_pub_->publish(*ack_msg);

      }
      else if (single_byte[0] == 0x56) {
        // ================== 处理第二类数据包：0xAA 0x56 ==================
        data.resize(PAYLOAD_SIZE_2);
        serial_driver_[0]->port()->receive(data);

        // 补回帧头
        data.insert(data.begin(), {0xAA, 0x56});

        // 假设你还没有写专门针对第二种包的 fromVector_2，
        // 因为结构体是 packed 的，这里直接使用标准且高效的 memcpy 内存拷贝进行解析！
        ReceivePacket packet2 = fromVector(data);
        const robot_serial::msg::Command::SharedPtr dt35_msg = std::make_shared<robot_serial::msg::Command>();
        dt35_msg->x = packet2.x;
        dt35_msg->y = packet2.y;
        dt35_pub_->publish(*dt35_msg);
       // RCLCPP_INFO(get_logger(), "%.3f,%.3f",dt35_msg->x,dt35_msg->y);
        // 现在你可以在这里使用 packet2.x 和 packet2.y
        

      }
      else {
        // ================== 脏数据处理 ==================
        // 如果 0xAA 后面跟着的既不是 0x55 也不是 0x56，
        // 说明刚才的 0xAA 只是数据里碰巧一样的值，并非真正的帧头。
        // 直接进入下一次循环，重新去寻找真正的 0xAA。
        continue;
      }

    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5, "串口读取异常 (Port 0): %s", ex.what());
      reopenPort();
    }
  }
}

/**
 * @brief 得到数据
*/
void ChassisSerialDriver::getData1(const robot_serial::msg::Command::SharedPtr cmd)
{
    cmd_x_ = cmd->x;
    cmd_y_ = cmd->y;
    cmd_yaw_ = cmd->yaw;
    spearhead_ = cmd->spearhead;
    block_ = cmd->block;
    stair_ = cmd->stair;
    area_ = cmd->area;
    dt35_ = cmd->dt35;
}

void ChassisSerialDriver::sendData()
{
  try {
    //RCLCPP_INFO(get_logger(), "prepare senddata:");

    // 内存零初始化，防止内存错乱
    SendPacket packet{};

    // 直接传递坐标和指令，不做速度转换
    packet.x = cmd_x_;
    packet.y = cmd_y_;
    packet.yaw = cmd_yaw_;
    packet.x1 = cmd_x_1;
    packet.y1 = cmd_y_1;
    packet.yaw1 = cmd_yaw_1;
    packet.spearhead = spearhead_;
    packet.block = block_;
    packet.stair = stair_;
    packet.area = area_;
    packet.dt35 = dt35_;
    // Bounds check
    if (serial_driver_.empty() || !serial_driver_[0]) {
      RCLCPP_WARN(get_logger(), "serial_driver_[0] not available");
      return;
    }

    std::vector<uint8_t> data = toVector(packet);
    serial_driver_[0]->port()->send(data);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}


/**
 * @brief 从config/serial_driver中获取串口参数
*/
void ChassisSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    for(int i=0;i<serial_count;i++){
    device_name_[i] = declare_parameter<std::string>("device_name_" + std::to_string(i), "");
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control parameter provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity parameter provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits parameter provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

/**
 * @brief 重新打开串口
*/
void ChassisSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    for (size_t i = 0; i < static_cast<size_t>(serial_count) && i < serial_driver_.size(); i++) {
      if (serial_driver_[i] && serial_driver_[i]->port()->is_open()) {
        serial_driver_[i]->port()->close();
      }
      if (serial_driver_[i]) {
        serial_driver_[i]->port()->open();
      }
    }
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
}
void ChassisSerialDriver::get_location(const nav_msgs::msg::Odometry::SharedPtr location)
{
    double qz = location->pose.pose.orientation.z;
    double qw = location->pose.pose.orientation.w;

    cmd_yaw_1 = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);

    double radar_x = location->pose.pose.position.x;
    double radar_y = location->pose.pose.position.y;

    // 雷达到底盘中心的偏移修正
    cmd_x_1 = radar_x - 0.36 * std::cos(cmd_yaw_1) + 0.27 * std::sin(cmd_yaw_1);
    cmd_y_1 = radar_y - 0.36 * std::sin(cmd_yaw_1) - 0.27 * std::cos(cmd_yaw_1);
}
}// namespace chassis_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(chassis_serial_driver::ChassisSerialDriver)

