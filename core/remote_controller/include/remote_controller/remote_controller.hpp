#include "rclcpp/rclcpp.hpp"

#include "rm_message/msg/remote_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "std_msgs/msg/bool.hpp"

#ifndef RMREMOTE_CONTROLLER_HPP
#define RMREMOTE_CONTROLLER_HPP

namespace RM_REMOTE_CONTROLLER {

enum class REMOTE_CONTROL_BUTTON {
    STOP,
    KEYL,
    KEYR,
    KEYB,
    PRESSL,
    PRESSR,
    PRESSMID,
    W,
    S,
    A,
    D,
    SHIFT,
    CTRL,
    Q,
    E,
    R,
    F,
    G,
    Z,
    X,
    C,
    V,
    B
};

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

    float max_x = 0;
    float max_y = 0;
    float max_z = 0;

    float last_x_ = 0;
    float last_y_ = 0;
    float last_z_ = 0;

    float delta_x = 0;
    float delta_y = 0;
    float delta_z = 0;

    std::string stop_button_;

    rclcpp::Time last_time_;

    // 储存某个按键上一次的状态，用于检测按键边沿
    std::unordered_map<REMOTE_CONTROL_BUTTON, uint8_t> last_button_states_ = {
        {REMOTE_CONTROL_BUTTON::STOP, 0},
        {REMOTE_CONTROL_BUTTON::KEYL, 0},
        {REMOTE_CONTROL_BUTTON::KEYR, 0},
        {REMOTE_CONTROL_BUTTON::KEYB, 0},
        {REMOTE_CONTROL_BUTTON::PRESSL, 0},
        {REMOTE_CONTROL_BUTTON::PRESSR, 0},
        {REMOTE_CONTROL_BUTTON::PRESSMID, 0},
        {REMOTE_CONTROL_BUTTON::W, 0},
        {REMOTE_CONTROL_BUTTON::S, 0},
        {REMOTE_CONTROL_BUTTON::A, 0},
        {REMOTE_CONTROL_BUTTON::D, 0},
        {REMOTE_CONTROL_BUTTON::SHIFT, 0},
        {REMOTE_CONTROL_BUTTON::CTRL, 0},
        {REMOTE_CONTROL_BUTTON::Q, 0},
        {REMOTE_CONTROL_BUTTON::E, 0},
        {REMOTE_CONTROL_BUTTON::R, 0},
        {REMOTE_CONTROL_BUTTON::F, 0},
        {REMOTE_CONTROL_BUTTON::G, 0},
        {REMOTE_CONTROL_BUTTON::Z, 0},
        {REMOTE_CONTROL_BUTTON::X, 0},
        {REMOTE_CONTROL_BUTTON::C, 0},
        {REMOTE_CONTROL_BUTTON::V, 0},
        {REMOTE_CONTROL_BUTTON::B, 0}
    };

    // 储存某个按钮是否被按下了，触发式
    std::unordered_map<REMOTE_CONTROL_BUTTON, bool> button_pressed_ = {
        {REMOTE_CONTROL_BUTTON::STOP, false},
        {REMOTE_CONTROL_BUTTON::KEYL, false},
        {REMOTE_CONTROL_BUTTON::KEYR, false},
        {REMOTE_CONTROL_BUTTON::KEYB, false},
        {REMOTE_CONTROL_BUTTON::PRESSL, false},
        {REMOTE_CONTROL_BUTTON::PRESSR, false},
        {REMOTE_CONTROL_BUTTON::PRESSMID, false},
        {REMOTE_CONTROL_BUTTON::W, false},
        {REMOTE_CONTROL_BUTTON::S, false},
        {REMOTE_CONTROL_BUTTON::A, false},
        {REMOTE_CONTROL_BUTTON::D, false},
        {REMOTE_CONTROL_BUTTON::SHIFT, false},
        {REMOTE_CONTROL_BUTTON::CTRL, false},
        {REMOTE_CONTROL_BUTTON::Q, false},
        {REMOTE_CONTROL_BUTTON::E, false},
        {REMOTE_CONTROL_BUTTON::R, false},
        {REMOTE_CONTROL_BUTTON::F, false},
        {REMOTE_CONTROL_BUTTON::G, false},
        {REMOTE_CONTROL_BUTTON::Z, false},
        {REMOTE_CONTROL_BUTTON::X, false},
        {REMOTE_CONTROL_BUTTON::C, false},
        {REMOTE_CONTROL_BUTTON::V, false},
        {REMOTE_CONTROL_BUTTON::B, false}
    };

    // 储存某个按钮的状态，其每被触发一次就为就从false变为true，再次触发就变为false
    std::unordered_map<REMOTE_CONTROL_BUTTON, bool> button_toggled_ = {
        {REMOTE_CONTROL_BUTTON::STOP, false},
        {REMOTE_CONTROL_BUTTON::KEYL, false},
        {REMOTE_CONTROL_BUTTON::KEYR, false},
        {REMOTE_CONTROL_BUTTON::KEYB, false},
        {REMOTE_CONTROL_BUTTON::PRESSL, false},
        {REMOTE_CONTROL_BUTTON::PRESSR, false},
        {REMOTE_CONTROL_BUTTON::PRESSMID, false},
        {REMOTE_CONTROL_BUTTON::W, false},
        {REMOTE_CONTROL_BUTTON::S, false},
        {REMOTE_CONTROL_BUTTON::A, false},
        {REMOTE_CONTROL_BUTTON::D, false},
        {REMOTE_CONTROL_BUTTON::SHIFT, false},
        {REMOTE_CONTROL_BUTTON::CTRL, false},
        {REMOTE_CONTROL_BUTTON::Q, false},
        {REMOTE_CONTROL_BUTTON::E, false},
        {REMOTE_CONTROL_BUTTON::R, false},
        {REMOTE_CONTROL_BUTTON::F, false},
        {REMOTE_CONTROL_BUTTON::G, false},
        {REMOTE_CONTROL_BUTTON::Z, false},
        {REMOTE_CONTROL_BUTTON::X, false},
        {REMOTE_CONTROL_BUTTON::C, false},
        {REMOTE_CONTROL_BUTTON::V, false},
        {REMOTE_CONTROL_BUTTON::B, false}
    };

    // 更新按钮状态
    void updateButtonStates(const rm_message::msg::RemoteControl::SharedPtr msg);

};

} // namespace RM_REMOTE_CONTROLLER

#endif // RMREMOTE_CONTROLLER_HPP