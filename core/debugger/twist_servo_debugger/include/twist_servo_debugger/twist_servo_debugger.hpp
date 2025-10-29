#ifndef TWIST_SERVO_DEBUGGER__TWIST_SERVO_DEBUGGER_HPP_
#define TWIST_SERVO_DEBUGGER__TWIST_SERVO_DEBUGGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rm_message/msg/remote_control.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <string>
#include <vector>

class TwistServoDebugger : public rclcpp::Node {
public:
  explicit TwistServoDebugger();
  virtual ~TwistServoDebugger();

private:
  struct Mapping {
    int channel{0};
    std::string axis; // e.g. "linear.x" or "angular.z"
    double max{0.5};
    bool invert{false};
  };

  void rcCallback(const rm_message::msg::RemoteControl::SharedPtr msg);
  double get_channel_value(const rm_message::msg::RemoteControl::SharedPtr &msg, int idx);
  void set_axis_value(geometry_msgs::msg::Twist &tw, const std::string &axis, double val);

  std::string remote_controller_topic_;
  std::string output_topic_;

  std::vector<Mapping> mappings_;

  rclcpp::Subscription<rm_message::msg::RemoteControl>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::string frame_id_;

  // 线程相关
  std::unique_ptr<std::thread> publish_thread_;
  bool stop_thread_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistStamped> rt_buffer_;
  void publish_loop();
};

#endif // TWIST_SERVO_DEBUGGER__TWIST_SERVO_DEBUGGER_HPP_
