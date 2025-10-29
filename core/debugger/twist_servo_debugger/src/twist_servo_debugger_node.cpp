#include "twist_servo_debugger/twist_servo_debugger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rm_message/msg/remote_control.hpp>

#include <string>
#include <algorithm>
#include <mutex>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

TwistServoDebugger::TwistServoDebugger() : Node("twist_servo_debugger"), stop_thread_(false) {
  this->declare_parameter<std::string>("remote_controller_topic", "/rm_manager/remote_control");
  this->declare_parameter<std::string>("output_topic", "/twist_servo_debugger/twist_stamped");
  this->declare_parameter<std::string>("frame_id", "base_link");

  remote_controller_topic_ = this->get_parameter("remote_controller_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();

  // list-based mapping: support either top-level keys (channels/axes/max/invert)
  // or namespaced keys under map.* (for backward compatibility)
  this->declare_parameter<std::vector<int64_t>>("channels", std::vector<int64_t>{});
  this->declare_parameter<std::vector<std::string>>("axes", std::vector<std::string>{});
  this->declare_parameter<std::vector<double>>("max", std::vector<double>{});
  this->declare_parameter<std::vector<bool>>("invert", std::vector<bool>{});

  auto channels = this->get_parameter("channels").as_integer_array();
  auto axes = this->get_parameter("axes").as_string_array();
  auto maxs = this->get_parameter("max").as_double_array();
  auto inverts = this->get_parameter("invert").as_bool_array();

  size_t n = std::max({channels.size(), axes.size(), maxs.size(), inverts.size()});
  for (size_t i = 0; i < n; ++i) {
    Mapping m;
    m.channel = (i < channels.size() ? static_cast<int>(channels[i]) : static_cast<int>(i));
    m.axis = (i < axes.size() && !axes[i].empty()) ? axes[i] : std::string();
    m.max = (i < maxs.size() ? maxs[i] : 0.5);
    m.invert = (i < inverts.size() ? inverts[i] : false);
    mappings_.push_back(m);
    RCLCPP_INFO(this->get_logger(), "Added mapping: channel=%d -> axis=%s (max=%.3f invert=%s)",
                m.channel, m.axis.c_str(), m.max, m.invert?"true":"false");
  }

  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
  frame_id_ = this->get_parameter("frame_id").as_string();

  sub_ = this->create_subscription<rm_message::msg::RemoteControl>(
    remote_controller_topic_, 10,
    std::bind(&TwistServoDebugger::rcCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "TwistServoDebugger initialized. Subscribing to %s, publishing to %s",
              remote_controller_topic_.c_str(), output_topic_.c_str());
              
  // 启动发布线程
  publish_thread_ = std::make_unique<std::thread>(&TwistServoDebugger::publish_loop, this);
}

TwistServoDebugger::~TwistServoDebugger() {
  stop_thread_ = true;
  if (publish_thread_ && publish_thread_->joinable()) {
    publish_thread_->join();
  }
}

void TwistServoDebugger::publish_loop() {
  rclcpp::Rate rate(1000);  // 1kHz
  while (rclcpp::ok() && !stop_thread_) {
    // 从实时缓冲区读取最新数据
    const auto & twist_msg = *rt_buffer_.readFromRT();
    // 发布消息
    twist_pub_->publish(twist_msg);
    rate.sleep();
  }
}

void TwistServoDebugger::rcCallback(const rm_message::msg::RemoteControl::SharedPtr msg) {
  // 创建新的 TwistStamped 消息
  geometry_msgs::msg::TwistStamped new_twist;
  new_twist.header.stamp = this->now();
  new_twist.header.frame_id = frame_id_;
  
  // 计算各轴的值
  for (auto &m : mappings_) {
    double raw = get_channel_value(msg, m.channel);
    double val = (raw - 1024.0) / 660.0 * m.max;
    if (m.invert) val = -val;
    if (!m.axis.empty()) set_axis_value(new_twist.twist, m.axis, val);
  }
  
  // 写入实时缓冲区
  rt_buffer_.writeFromNonRT(new_twist);
}

double TwistServoDebugger::get_channel_value(const rm_message::msg::RemoteControl::SharedPtr &msg, int idx) {
  switch (idx) {
    case 0: return msg->chanal0;
    case 1: return msg->chanal1;
    case 2: return msg->chanal2;
    case 3: return msg->chanal3;
    default: return 1024.0;
  }
}

void TwistServoDebugger::set_axis_value(geometry_msgs::msg::Twist &tw, const std::string &axis, double val) {
  // accept formats like "linear.x" or "angular.z"
  if (axis == "linear.x") tw.linear.x = static_cast<float>(val);
  else if (axis == "linear.y") tw.linear.y = static_cast<float>(val);
  else if (axis == "linear.z") tw.linear.z = static_cast<float>(val);
  else if (axis == "angular.x") tw.angular.x = static_cast<float>(val);
  else if (axis == "angular.y") tw.angular.y = static_cast<float>(val);
  else if (axis == "angular.z") tw.angular.z = static_cast<float>(val);
  else RCLCPP_WARN(this->get_logger(), "Unknown axis '%s' in mapping, ignoring", axis.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistServoDebugger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
