#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rm_message/msg/general_message.hpp"
#include "rm_message/msg/send_message.hpp"
#include "rm_message/msg/remote_control.hpp"

#include "rm_manager/uart_driver.hpp"

#include <queue>
#include <atomic>

# ifndef RM_MANAGER_HPP
# define RM_MANAGER_HPP

namespace RMManager {

// 帧头定义
struct FrameHeader {
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
}__attribute__((packed));

struct RemoteControlData{
    uint8_t head1;
    uint8_t head2;
    uint16_t chanal0 : 11;
    uint16_t chanal1 : 11;
    uint16_t chanal2 : 11;
    uint16_t chanal3 : 11;
    uint8_t cut : 2;
    uint8_t stop : 1;
    uint8_t keyl : 1;
    uint8_t keyr : 1;
    uint16_t wheel : 11;
    uint8_t keyb : 1;
    uint16_t mousex : 16;
    uint16_t mousey : 16;
    uint16_t mousez : 16;
    uint8_t pressl : 2;
    uint8_t pressr : 2;
    uint8_t pressmid : 2;
    struct{
        uint8_t w : 1;
        uint8_t s : 1;
        uint8_t a : 1;
        uint8_t d : 1;
        uint8_t shift : 1;
        uint8_t ctrl : 1;
        uint8_t q : 1;
        uint8_t e : 1;
        uint8_t r : 1;
        uint8_t f : 1;
        uint8_t g : 1;
        uint8_t z : 1;
        uint8_t x : 1;
        uint8_t c : 1;
        uint8_t v : 1;
        uint8_t b : 1;
    }keyboards;
    uint16_t crc : 16;
}__attribute__((packed));

class RMManagerNode : public rclcpp::Node {

public:
    RMManagerNode(std::string name = "rm_manager");
    ~RMManagerNode();

private:

    // 图传链路
    std::shared_ptr<SerialCommunicator> image_uart_;

    // 裁判系统链路
    std::shared_ptr<SerialCommunicator> referee_uart_;

    /**
     * @brief 处理接受到的数据的回调函数，分割数据
     * 
     * @param data  接收到的数据
     * @param link_status 链路状态的标志位
     */
    void _read_callback(const std::vector<uint8_t>& data, std::atomic<bool>& link_status);

    // 图传链路是否发送了消息
    std::atomic<bool> image_send_{true};

    // 裁判系统链路是否发送了消息
    std::atomic<bool> referee_send_{true};

    // 监测图传链路状态的线程
    std::unique_ptr<std::thread> image_check_thread_;
    // 监测裁判系统链路状态的线程
    std::unique_ptr<std::thread> referee_check_thread_;

    // 图传链路状态的pub
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> image_status_pub_;
    // 裁判系统链路状态的pub
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> referee_status_pub_;

    // 处理接受的数据的pub
    std::map<uint16_t, std::shared_ptr<rclcpp::Publisher<rm_message::msg::GeneralMessage>>> general_pubs_;
    // 保护general_pubs_的互斥锁
    std::mutex general_pubs_mutex_;

    // 遥控器数据的pub
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RemoteControl>> remoto_control_pub_;

    // remote_control_pub_的topic名称
    std::string remote_control_topic_ = "/remote_control";

    // custom_command_topics参数
    std::vector<std::string> custom_command_topics_;
    // custom_command_codes参数
    std::vector<int64_t> custom_command_codes_;

    /**
     * @brief 得到对应command id 的pub
     * 
     * @param header 
     * @return std::weak_ptr<rclcpp::Publisher<rm_message::msg::GeneralMessage>>
     */
    std::weak_ptr<rclcpp::Publisher<rm_message::msg::GeneralMessage>> _get_general_pub(uint16_t header);

    // 接受数据的sub
    std::shared_ptr<rclcpp::Subscription<rm_message::msg::SendMessage>> send_sub_;

    /**
     * @brief 处理接受到的数据
     * 
     */
    void _send_sub_callback(const rm_message::msg::SendMessage::SharedPtr msg);

    /**
     * @brief 处理来自图传的特殊的消息
     * 
     * @param data 
     * @return true 
     * @return false 
     */
    bool _process_image_own_message(const std::vector<uint8_t>& data);

    std::map<int, std::string> default_command_topics = {
        {0x0001, "game_status_data"},
        {0x0002, "game_result_data"},
        {0x0003, "robot_hp_data"},
        {0x0101, "field_event_data"},
        {0x0104, "referee_warning_data"},
        {0x0105, "dart_launch_data"},
        {0x0201, "robot_performance_data"},
        {0x0202, "chassis_buffer_heat_data"},
        {0x0203, "robot_position_data"},
        {0x0204, "robot_buff_chassis_energy_data"},
        {0x0206, "damage_status_data"},
        {0x0207, "realtime_shooting_data"},
        {0x0208, "allowed_projectile_data"},
        {0x0209, "robot_rfid_module_status"},
        {0x020A, "dart_client_command_data"},
        {0x020B, "ground_robot_position_data"},
        {0x020C, "radar_mark_progress_data"},
        {0x020D, "sentry_auto_decision_sync"},
        {0x020E, "radar_auto_decision_sync"},
        {0x0301, "robot_interaction_data"},
        {0x0302, "custom_controller_to_robot_data"},
        {0x0303, "client_minimap_interaction_data"},
        {0x0304, "keyboard_mouse_control_data"},
        {0x0305, "client_minimap_receive_radar_data"},
        {0x0306, "custom_controller_to_client_data"},
        {0x0307, "client_minimap_receive_path_data"},
        {0x0308, "client_minimap_receive_robot_data"},
        {0x0309, "custom_controller_receive_robot_data"}
    };

}; // class RMManagerNode

rm_message::msg::RemoteControl _remote_control_data_to_msg(const RemoteControlData& data);

} // namespace RMManager

# endif // RM_MANAGER_HPP