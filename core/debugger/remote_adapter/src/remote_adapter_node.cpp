#include "remote_adapter/remote_adapter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rm_message/msg/remote_control.hpp>

#include <string>
#include <memory>

RemoteAdapter::RemoteAdapter() : Node("remote_adapter") {
  // topics and counts
  this->declare_parameter<std::string>("remote_controller_topic", "/rm_manager/remote_control");

  // enable topics (same config as remote_controller)
  this->declare_parameter<std::string>("chasis_enable_topic", "/enable_chasis");
  this->declare_parameter<std::string>("arm_enable_topic", "/enable_arm");

  remote_controller_topic_ = this->get_parameter("remote_controller_topic").as_string();
  chasis_enable_topic_ = this->get_parameter("chasis_enable_topic").as_string();
  arm_enable_topic_ = this->get_parameter("arm_enable_topic").as_string();

  // Try YAML list-based config first: map.channels, map.topics, map.max, map.invert
  this->declare_parameter<std::vector<int64_t>>("map.channels", std::vector<int64_t>{});
  this->declare_parameter<std::vector<std::string>>("map.topics", std::vector<std::string>{});
  this->declare_parameter<std::vector<double>>("map.max", std::vector<double>{});
  this->declare_parameter<std::vector<bool>>("map.invert", std::vector<bool>{});

  auto channels = this->get_parameter("map.channels").as_integer_array();
  auto topics = this->get_parameter("map.topics").as_string_array();
  auto maxs = this->get_parameter("map.max").as_double_array();
  auto inverts = this->get_parameter("map.invert").as_bool_array();

  bool use_list_mode = !channels.empty() || !topics.empty() || !maxs.empty() || !inverts.empty();

  if (use_list_mode) {
    size_t n = std::max({channels.size(), topics.size(), maxs.size(), inverts.size()});
    if (n == 0) n = std::max<size_t>(channels.size(), topics.size());
    for (size_t i = 0; i < n; ++i) {
      Mapping m;
      m.channel = (i < channels.size() ? static_cast<int>(channels[i]) : static_cast<int>(i));
      m.topic = (i < topics.size() && !topics[i].empty()) ? topics[i] : ("/remote_adapter/chan" + std::to_string(i));
      m.max = (i < maxs.size() ? maxs[i] : 0.5);
      m.invert = (i < inverts.size() ? inverts[i] : false);
      if (!m.topic.empty()) {
        m.pub = this->create_publisher<std_msgs::msg::Float32>(m.topic, 10);
      }
      mappings_.push_back(std::move(m));
      RCLCPP_INFO(this->get_logger(), "Added mapping (list): channel=%d -> topic=%s (max=%.3f invert=%s)",
                  m.channel, m.topic.c_str(), m.max, m.invert ? "true" : "false");
    }
  }

  // create enable publishers if topics provided
  if (!chasis_enable_topic_.empty()) {
    chasis_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(chasis_enable_topic_, 10);
    RCLCPP_INFO(this->get_logger(), "Publishing chasis enable to %s", chasis_enable_topic_.c_str());
  }
  if (!arm_enable_topic_.empty()) {
    arm_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(arm_enable_topic_, 10);
    RCLCPP_INFO(this->get_logger(), "Publishing arm enable to %s", arm_enable_topic_.c_str());
  }

  // subscriber
  sub_ = this->create_subscription<rm_message::msg::RemoteControl>(
    remote_controller_topic_, 10,
    std::bind(&RemoteAdapter::rcCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "RemoteAdapter initialized. Subscribing to %s", remote_controller_topic_.c_str());
}

void RemoteAdapter::rcCallback(const rm_message::msg::RemoteControl::SharedPtr msg) {
  // publish mapped float topics
  for (auto &m : mappings_) {
    if (!m.pub) continue;
    double raw = get_channel_value(msg, m.channel);
    double val = (raw - 1024.0) / 660.0 * m.max;
    if (m.invert) val = -val;
    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(val);
    m.pub->publish(out);
  }

  // enable topics (same logic as remote_controller)
  if (chasis_enable_pub_) {
    std_msgs::msg::Bool b;
    int cut = msg->cut;
    b.data = (cut == 1 || cut == 2);
    chasis_enable_pub_->publish(b);
  }
  if (arm_enable_pub_) {
    std_msgs::msg::Bool b;
    int cut = msg->cut;
    b.data = (cut == 2);
    arm_enable_pub_->publish(b);
  }
}

double RemoteAdapter::get_channel_value(const rm_message::msg::RemoteControl::SharedPtr &msg, int idx) {
  switch (idx) {
    case 0: return msg->chanal0;
    case 1: return msg->chanal1;
    case 2: return msg->chanal2;
    case 3: return msg->chanal3;
    default: return 1024.0; // center
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RemoteAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
