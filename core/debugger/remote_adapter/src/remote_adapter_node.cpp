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
  this->declare_parameter<std::vector<int64_t>>("channels", std::vector<int64_t>{});
  this->declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
  this->declare_parameter<std::vector<double>>("max", std::vector<double>{});
  this->declare_parameter<std::vector<bool>>("invert", std::vector<bool>{});

  auto channels = this->get_parameter("channels").as_integer_array();
  auto topics = this->get_parameter("topics").as_string_array();
  auto maxs = this->get_parameter("max").as_double_array();
  auto inverts = this->get_parameter("invert").as_bool_array();


  // 检查参数是否长度匹配
  if (channels.size() != topics.size() || channels.size() != maxs.size() || channels.size() != inverts.size()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter length mismatch: channels(%zu), topics(%zu), max(%zu), invert(%zu)",
                 channels.size(), topics.size(), maxs.size(), inverts.size());
    throw std::runtime_error("Parameter length mismatch");
  }

    size_t n = std::max({channels.size(), topics.size(), maxs.size(), inverts.size()});
    if (n == 0) n = std::max<size_t>(channels.size(), topics.size());
    for (size_t i = 0; i < n; ++i) {
        Mapping m;
        m.channel = static_cast<int>(channels[i]);
        m.topic = topics[i];
        m.max = maxs[i];
        m.invert = inverts[i];
        if (!m.topic.empty()) {
            m.pub = this->create_publisher<std_msgs::msg::Float32>(m.topic, 10);
        }
        mappings_.push_back(std::move(m));
        RCLCPP_INFO(this->get_logger(), "Added mapping (list): channel=%d -> topic=%s (max=%.3f invert=%s)",
                    m.channel, m.topic.c_str(), m.max, m.invert ? "true" : "false");
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
  
  // 初始化实时缓冲区
  ChannelData initial_data;
  initial_data.channel_values.resize(mappings_.size(), 0.0f);
  initial_data.chasis_enable = false;
  initial_data.arm_enable = false;
  rt_buffer_.writeFromNonRT(initial_data);

  // 启动发布线程
  publish_thread_ = std::make_unique<std::thread>(&RemoteAdapter::publish_loop, this);
}

RemoteAdapter::~RemoteAdapter() {
  stop_thread_ = true;
  if (publish_thread_ && publish_thread_->joinable()) {
    publish_thread_->join();
  }
}

void RemoteAdapter::publish_loop() {
  rclcpp::Rate rate(1000);  // 1kHz
  while (rclcpp::ok() && !stop_thread_) {
    const auto& data = *rt_buffer_.readFromRT();
    
    // 发布通道数据
    for (size_t i = 0; i < mappings_.size(); ++i) {
      if (mappings_[i].pub) {
        std_msgs::msg::Float32 msg;
        msg.data = data.channel_values[i];
        mappings_[i].pub->publish(msg);
      }
    }

    // 发布使能状态
    if (chasis_enable_pub_) {
      std_msgs::msg::Bool msg;
      msg.data = data.chasis_enable;
      chasis_enable_pub_->publish(msg);
    }
    if (arm_enable_pub_) {
      std_msgs::msg::Bool msg;
      msg.data = data.arm_enable;
      arm_enable_pub_->publish(msg);
    }

    rate.sleep();
  }
}

void RemoteAdapter::rcCallback(const rm_message::msg::RemoteControl::SharedPtr msg) {
  // 准备新的通道数据
  RemoteAdapter::ChannelData new_data;
  new_data.channel_values.resize(mappings_.size());

  // 计算所有通道的值
  for (size_t i = 0; i < mappings_.size(); ++i) {
    if (!mappings_[i].pub) continue;
    double raw = get_channel_value(msg, mappings_[i].channel);
    double val = (raw - 1024.0) / 660.0 * mappings_[i].max;
    if (mappings_[i].invert) val = -val;
    new_data.channel_values[i] = static_cast<float>(val);
  }

  // 计算使能状态
  int cut = msg->cut;
  new_data.chasis_enable = (cut == 1 || cut == 2);
  new_data.arm_enable = (cut == 2);

  // 写入实时缓冲区
  rt_buffer_.writeFromNonRT(new_data);
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
