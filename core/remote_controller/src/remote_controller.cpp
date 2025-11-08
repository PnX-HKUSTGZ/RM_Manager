#include "remote_controller/remote_controller.hpp"

namespace RM_REMOTE_CONTROLLER {

RemoteController::RemoteController(std::string name) : rclcpp::Node(name) {
    this->declare_parameter<std::string>("cmd_vel_topic", "/mcknum_controller/reference");
    this->declare_parameter<std::string>("remote_controller_topic", "/rm_manager/remote_control");
    this->declare_parameter<std::string>("chasis_enable_topic", "/enable_chasis");
    this->declare_parameter<std::string>("arm_enable_topic", "/enable_arm");

    this->declare_parameter<double>("max_x", 0.5);
    this->declare_parameter<double>("max_y", 0.5);
    this->declare_parameter<double>("max_z", 0.5);


    this->declare_parameter<double>("delta_x", 1000.0);
    this->declare_parameter<double>("delta_y", 1000.0);
    this->declare_parameter<double>("delta_z", 1000.0);

    try{
        max_x = this->get_parameter("max_x").as_double();
        max_y = this->get_parameter("max_y").as_double();
        max_z = this->get_parameter("max_z").as_double();
        delta_x = this->get_parameter("delta_x").as_double();
        delta_y = this->get_parameter("delta_y").as_double();
        delta_z = this->get_parameter("delta_z").as_double();
    }
    catch (const rclcpp::ParameterTypeException & e) {
        RCLCPP_ERROR(this->get_logger(), "Parameter type error: %s", e.what());
        throw;
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while getting parameters: %s", e.what());
        throw;
    }

    // 检查参数值是否合理
    if (max_x < 0 || max_y < 0 || max_z < 0) {
        RCLCPP_ERROR(this->get_logger(), "max_x, max_y, and max_z must be non-negative.");
        throw std::invalid_argument("Invalid max_x, max_y, or max_z parameter");
    }
    if (delta_x <= 0 || delta_y <= 0 || delta_z <= 0) {
        RCLCPP_ERROR(this->get_logger(), "delta_x, delta_y, and delta_z must be positive.");
        throw std::invalid_argument("Invalid delta_x, delta_y, or delta_z parameter");
    }

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
    // 输出参数值
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  cmd_vel_topic: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  remote_controller_topic: %s", remote_controller_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  chasis_enable_topic: %s", chasis_enable_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  arm_enable_topic: %s", arm_enable_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  max_x: %.2f", max_x);
    RCLCPP_INFO(this->get_logger(), "  max_y: %.2f", max_y);
    RCLCPP_INFO(this->get_logger(), "  max_z: %.2f", max_z);
    RCLCPP_INFO(this->get_logger(), "  delta_x: %.2f", delta_x);
    RCLCPP_INFO(this->get_logger(), "  delta_y: %.2f", delta_y);
    RCLCPP_INFO(this->get_logger(), "  delta_z: %.2f", delta_z);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);
    cmd_vel_sub_ = this->create_subscription<rm_message::msg::RemoteControl>(
        remote_controller_topic_, 10,
        std::bind(&RemoteController::cmdVelCallback, this, std::placeholders::_1)
    );
    chasis_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(chasis_enable_topic_, 10);
    arm_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(arm_enable_topic_, 10);

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "RemoteController node initialized. Subscribing to %s, publishing to %s", remote_controller_topic_.c_str(), cmd_vel_topic_.c_str());
}

RemoteController::~RemoteController() {}

void RemoteController::cmdVelCallback(const rm_message::msg::RemoteControl::SharedPtr msg) {
    updateButtonStates(msg);
    sendVel(msg);
    sendEnableChasis(msg);
    sendEnableArm(msg);
}

void RemoteController::sendVel(const rm_message::msg::RemoteControl::SharedPtr msg) {

    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";

    // 限制加速度
    float desired_x = static_cast<double>(msg->chanal2 - 1024) / 660 * max_x;
    float desired_y = -static_cast<double>(msg->chanal3 - 1024) / 660 * max_y;
    float desired_z = -static_cast<double>(msg->chanal0 - 1024) / 660 * max_z;

    rclcpp::Time current_time = this->now();
    double time_diff = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // 限制加速度 允许急停
    if (std::abs(desired_x) >= 1e-4 && std::abs(desired_x - last_x_) > delta_x*time_diff) {
        desired_x = last_x_ + (desired_x > last_x_ ? delta_x*time_diff : -delta_x*time_diff);
    }
    if (std::abs(desired_y) >= 1e-4 && std::abs(desired_y - last_y_) > delta_y*time_diff) {
        desired_y = last_y_ + (desired_y > last_y_ ? delta_y*time_diff : -delta_y*time_diff);
    }
    if (std::abs(desired_z) >= 1e-4 && std::abs(desired_z - last_z_) > delta_z*time_diff) {
        desired_z = last_z_ + (desired_z > last_z_ ? delta_z*time_diff : -delta_z*time_diff);
    }

    twist_msg.twist.linear.x = desired_x;
    twist_msg.twist.linear.y = desired_y;
    twist_msg.twist.angular.z = desired_z;

    last_x_ = twist_msg.twist.linear.x;
    last_y_ = twist_msg.twist.linear.y;
    last_z_ = twist_msg.twist.angular.z;

    if (button_toggled_[REMOTE_CONTROL_BUTTON::KEYB]) {
        return;
    }

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

void RemoteController::updateButtonStates(const rm_message::msg::RemoteControl::SharedPtr msg) {


    // 检查某个按钮是否被按下的逻辑实现
    button_pressed_[REMOTE_CONTROL_BUTTON::STOP] = (last_button_states_[REMOTE_CONTROL_BUTTON::STOP] == 0 && msg->stop == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::KEYL] = (last_button_states_[REMOTE_CONTROL_BUTTON::KEYL] == 0 && msg->keyl == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::KEYR] = (last_button_states_[REMOTE_CONTROL_BUTTON::KEYR] == 0 && msg->keyr == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::KEYB] = (last_button_states_[REMOTE_CONTROL_BUTTON::KEYB] == 0 && msg->keyb == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::PRESSL] = (last_button_states_[REMOTE_CONTROL_BUTTON::PRESSL] == 0 && msg->pressl == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::PRESSR] = (last_button_states_[REMOTE_CONTROL_BUTTON::PRESSR] == 0 && msg->pressr == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::PRESSMID] = (last_button_states_[REMOTE_CONTROL_BUTTON::PRESSMID] == 0 && msg->pressmid == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::W] = (last_button_states_[REMOTE_CONTROL_BUTTON::W] == 0 && msg->w == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::S] = (last_button_states_[REMOTE_CONTROL_BUTTON::S] == 0 && msg->s == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::A] = (last_button_states_[REMOTE_CONTROL_BUTTON::A] == 0 && msg->a == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::D] = (last_button_states_[REMOTE_CONTROL_BUTTON::D] == 0 && msg->d == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::SHIFT] = (last_button_states_[REMOTE_CONTROL_BUTTON::SHIFT] == 0 && msg->shift == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::CTRL] = (last_button_states_[REMOTE_CONTROL_BUTTON::CTRL] == 0 && msg->ctrl == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::Q] = (last_button_states_[REMOTE_CONTROL_BUTTON::Q] == 0 && msg->q == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::E] = (last_button_states_[REMOTE_CONTROL_BUTTON::E] == 0 && msg->e == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::R] = (last_button_states_[REMOTE_CONTROL_BUTTON::R] == 0 && msg->r == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::F] = (last_button_states_[REMOTE_CONTROL_BUTTON::F] == 0 && msg->f == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::G] = (last_button_states_[REMOTE_CONTROL_BUTTON::G] == 0 && msg->g == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::Z] = (last_button_states_[REMOTE_CONTROL_BUTTON::Z] == 0 && msg->z == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::X] = (last_button_states_[REMOTE_CONTROL_BUTTON::X] == 0 && msg->x == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::C] = (last_button_states_[REMOTE_CONTROL_BUTTON::C] == 0 && msg->c == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::V] = (last_button_states_[REMOTE_CONTROL_BUTTON::V] == 0 && msg->v == 1);
    button_pressed_[REMOTE_CONTROL_BUTTON::B] = (last_button_states_[REMOTE_CONTROL_BUTTON::B] == 0 && msg->b == 1);

    // 更新按钮状态的逻辑实现
    last_button_states_[REMOTE_CONTROL_BUTTON::STOP] = msg->stop;
    last_button_states_[REMOTE_CONTROL_BUTTON::KEYL] = msg->keyl;
    last_button_states_[REMOTE_CONTROL_BUTTON::KEYR] = msg->keyr;
    last_button_states_[REMOTE_CONTROL_BUTTON::KEYB] = msg->keyb;
    last_button_states_[REMOTE_CONTROL_BUTTON::PRESSL] = msg->pressl;
    last_button_states_[REMOTE_CONTROL_BUTTON::PRESSR] = msg->pressr;
    last_button_states_[REMOTE_CONTROL_BUTTON::PRESSMID] = msg->pressmid;
    last_button_states_[REMOTE_CONTROL_BUTTON::W] = msg->w;
    last_button_states_[REMOTE_CONTROL_BUTTON::S] = msg->s;
    last_button_states_[REMOTE_CONTROL_BUTTON::A] = msg->a;
    last_button_states_[REMOTE_CONTROL_BUTTON::D] = msg->d;
    last_button_states_[REMOTE_CONTROL_BUTTON::SHIFT] = msg->shift;
    last_button_states_[REMOTE_CONTROL_BUTTON::CTRL] = msg->ctrl;
    last_button_states_[REMOTE_CONTROL_BUTTON::Q] = msg->q;
    last_button_states_[REMOTE_CONTROL_BUTTON::E] = msg->e;
    last_button_states_[REMOTE_CONTROL_BUTTON::R] = msg->r;
    last_button_states_[REMOTE_CONTROL_BUTTON::F] = msg->f;
    last_button_states_[REMOTE_CONTROL_BUTTON::G] = msg->g;
    last_button_states_[REMOTE_CONTROL_BUTTON::Z] = msg->z;
    last_button_states_[REMOTE_CONTROL_BUTTON::X] = msg->x;
    last_button_states_[REMOTE_CONTROL_BUTTON::C] = msg->c;
    last_button_states_[REMOTE_CONTROL_BUTTON::V] = msg->v;
    last_button_states_[REMOTE_CONTROL_BUTTON::B] = msg->b;

    // 更新button_toggled_
    for (auto &pair : button_pressed_) {
        REMOTE_CONTROL_BUTTON button = pair.first;
        if (pair.second) { // 如果按钮被按下
            button_toggled_[button] = !button_toggled_[button]; // 切换状态
        }
    }
}

} // namespace RM_REMOTE_CONTROLLER