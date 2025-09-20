#include "rclcpp/rclcpp.hpp"

#include "rm_message/msg/remote_control.hpp"
#include "geometry_msgs/msg/twist.hpp"

#ifndef RMREMOTE_CONTROLLER_HPP
#define RMREMOTE_CONTROLLER_HPP

namespace RM_REMOTE_CONTROLLER {

class RemoteController : public rclcpp::Node{
public:
    RemoteController(std::string  name = "remote_controller");
    ~RemoteController();
private:

    std::string cmd_vel_topic_;
    std::string remote_controller_topic_;
    rclcpp::Subscription<rm_message::msg::RemoteControl>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void cmdVelCallback(const rm_message::msg::RemoteControl::SharedPtr msg);

};

} // namespace RM_REMOTE_CONTROLLER

#endif // RMREMOTE_CONTROLLER_HPP