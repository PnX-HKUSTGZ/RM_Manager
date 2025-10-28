#ifndef REMOTE_ADAPTER__REMOTE_ADAPTER_HPP_
#define REMOTE_ADAPTER__REMOTE_ADAPTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rm_message/msg/remote_control.hpp>

#include <string>
#include <vector>

class RemoteAdapter : public rclcpp::Node {
public:
  explicit RemoteAdapter();

private:
  struct Mapping {
    int channel{0};
    std::string topic;
    double max{0.5};
    bool invert{false};
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub{nullptr};
  };

  void rcCallback(const rm_message::msg::RemoteControl::SharedPtr msg);
  double get_channel_value(const rm_message::msg::RemoteControl::SharedPtr &msg, int idx);

  std::string remote_controller_topic_;
  std::string chasis_enable_topic_;
  std::string arm_enable_topic_;

  std::vector<Mapping> mappings_;

  rclcpp::Subscription<rm_message::msg::RemoteControl>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr chasis_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_enable_pub_;
};

#endif  // REMOTE_ADAPTER__REMOTE_ADAPTER_HPP_
