#include "rclcpp/rclcpp.hpp"

#include "rm_message/msg/remote_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "std_msgs/msg/bool.hpp"

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
    std::string chasis_enable_topic_;
    std::string arm_enable_topic_;
    rclcpp::Subscription<rm_message::msg::RemoteControl>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr chasis_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_enable_pub_;

    void cmdVelCallback(const rm_message::msg::RemoteControl::SharedPtr msg);

    void sendVel(const rm_message::msg::RemoteControl::SharedPtr msg);

    void sendEnableChasis(const rm_message::msg::RemoteControl::SharedPtr msg);

    void sendEnableArm(const rm_message::msg::RemoteControl::SharedPtr msg);

};

} // namespace RM_REMOTE_CONTROLLER

#endif // RMREMOTE_CONTROLLER_HPP