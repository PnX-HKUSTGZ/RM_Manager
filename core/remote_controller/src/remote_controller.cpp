#include "remote_controller/remote_controller.hpp"

namespace RM_REMOTE_CONTROLLER {

RemoteController::RemoteController(std::string name) : rclcpp::Node(name) {
    this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    this->declare_parameter<std::string>("remote_controller_topic", "/rm_manager/remote_control");
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    remote_controller_topic_ = this->get_parameter("remote_controller_topic").as_string();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    cmd_vel_sub_ = this->create_subscription<rm_message::msg::RemoteControl>(
        remote_controller_topic_, 10,
        std::bind(&RemoteController::cmdVelCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "RemoteController node initialized. Subscribing to %s, publishing to %s", remote_controller_topic_.c_str(), cmd_vel_topic_.c_str());
}

RemoteController::~RemoteController() {}

void RemoteController::cmdVelCallback(const rm_message::msg::RemoteControl::SharedPtr msg) {
    auto twist_msg = geometry_msgs::msg::Twist();
    // Assuming chanal1 is linear x and chanal2 is angular z
    twist_msg.linear.x = static_cast<double>(msg->chanal2 - 1024) / 660 * 0.5; // Scale as needed
    twist_msg.linear.y = static_cast<double>(msg->chanal3 - 1024) / 660 * 0.5; // Scale as needed
    twist_msg.angular.z = static_cast<double>(msg->chanal0 - 1024) / 660 * 0.5; // Scale as needed
    cmd_vel_pub_->publish(twist_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published cmd_vel: linear.x=%.3f, angular.z=%.3f", twist_msg.linear.x, twist_msg.angular.z);
}

} // namespace RM_REMOTE_CONTROLLER