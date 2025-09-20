#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rm_message/msg/general_message.hpp"
#include "rm_message/msg/send_message.hpp"
#include "rm_message/msg/remoto_controll.hpp"

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

    // 处理接受到的数据的回调函数
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
    std::shared_ptr<rclcpp::Publisher<rm_message::msg::RemotoControll>> remoto_controll_pub_;

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

}; // class RMManagerNode

rm_message::msg::RemotoControll _remote_control_data_to_msg(const RemoteControlData& data);

} // namespace RMManager

# endif // RM_MANAGER_HPP