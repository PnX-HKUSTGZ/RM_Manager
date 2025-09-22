#include "remote_controller/remote_controller.hpp"

namespace RM_REMOTE_CONTROLLER {

RemoteController::RemoteController(std::string name) : rclcpp::Node(name) {
    this->declare_parameter<std::string>("cmd_vel_topic", "/mcknum_controller/reference");
    this->declare_parameter<std::string>("remote_controller_topic", "/rm_manager/remote_control");
    this->declare_parameter<std::string>("chasis_enable_topic", "/enable_chasis");
    this->declare_parameter<std::string>("arm_enable_topic", "/enable_arm");

    try{
        cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
        remote_controller_topic_ = this->get_parameter("remote_controller_topic").as_string();
        chasis_enable_topic_ = this->get_parameter("chasis_enable_topic").as_string();
        arm_enable_topic_ = this->get_parameter("arm_enable_topic").as_string();
    }
    catch (const rclcpp::ParameterTypeException & e) {
        RCLCPP_ERROR(this->get_logger(), "Parameter type error: %s", e.what());
        throw;
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while getting parameters: %s", e.what());
        throw;
    }

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);
    cmd_vel_sub_ = this->create_subscription<rm_message::msg::RemoteControl>(
        remote_controller_topic_, 10,
        std::bind(&RemoteController::cmdVelCallback, this, std::placeholders::_1)
    );
    chasis_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(chasis_enable_topic_, 10);
    arm_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(arm_enable_topic_, 10);


    RCLCPP_INFO(this->get_logger(), "RemoteController node initialized. Subscribing to %s, publishing to %s", remote_controller_topic_.c_str(), cmd_vel_topic_.c_str());
}

RemoteController::~RemoteController() {}

void RemoteController::cmdVelCallback(const rm_message::msg::RemoteControl::SharedPtr msg) {
    sendVel(msg);
    sendEnableChasis(msg);
    sendEnableArm(msg);
}

void RemoteController::sendVel(const rm_message::msg::RemoteControl::SharedPtr msg) {
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";
    // Assuming chanal1 is linear x and chanal2 is angular z
    twist_msg.twist.linear.x = static_cast<double>(msg->chanal2 - 1024) / 660 * 0.5; // Scale as needed
    twist_msg.twist.linear.y = -static_cast<double>(msg->chanal3 - 1024) / 660 * 0.5; // Scale as needed
    twist_msg.twist.angular.z = -static_cast<double>(msg->chanal0 - 1024) / 660 * 0.5; // Scale as needed
    cmd_vel_pub_->publish(twist_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published cmd_vel: linear.x=%.3f, angular.z=%.3f", twist_msg.twist.linear.x, twist_msg.twist.angular.z);
}

void RemoteController::sendEnableChasis(const rm_message::msg::RemoteControl::SharedPtr msg) {
    auto enable_msg = std_msgs::msg::Bool();
    enable_msg.data = (std::to_string(msg->cut) == "1" || std::to_string(msg->cut) == "2");
    chasis_enable_pub_->publish(enable_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published chasis_enable: %s", enable_msg.data ? "true" : "false");
}

void RemoteController::sendEnableArm(const rm_message::msg::RemoteControl::SharedPtr msg) {
    auto enable_msg = std_msgs::msg::Bool();
    enable_msg.data = (std::to_string(msg->cut) == "2");
    arm_enable_pub_->publish(enable_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published arm_enable: %s", enable_msg.data ? "true" : "false");
}

} // namespace RM_REMOTE_CONTROLLER