#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rm_message/msg/general_message.hpp"
#include "rm_message/msg/send_message.hpp"

#include "rm_manager/uart_driver.hpp"

#include <queue>
#include <atomic>

# ifndef RM_MANAGER_HPP
# define RM_MANAGER_HPP

namespace RMManager {

class RMManagerNode : public rclcpp::Node {

public:
    RMManagerNode(std::string name = "rm_manager") : Node(name) {
    }

private:
    
    // 来自不同链路的消息队列，储存的是原始消息
    std::queue<std::vector<uint8_t>> infor_queue_;
    // 保护消息队列的互斥锁
    std::mutex infor_queue_mutex_;

    // 图传链路
    std::shared_ptr<SerialCommunicator> image_uart_;

    // 裁判系统链路
    std::shared_ptr<SerialCommunicator> referee_uart_;

    // 处理接受到的数据的回调函数
    void _read_callback(const std::vector<uint8_t>& data, std::atomic<bool>& link_status);

    // 图传链路在线状态
    std::atomic<bool> image_online_{true};

    // 裁判系统链路在线状态
    std::atomic<bool> referee_online_{true};

    // 监测图传链路状态的线程
    std::unique_ptr<std::thread> image_check_thread_;
    // 监测裁判系统链路状态的线程
    std::unique_ptr<std::thread> referee_check_thread_;

    // 图传链路状态的pub
    std::unique_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> image_status_pub_;
    // 裁判系统链路状态的pub
    std::unique_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> referee_status_pub_;

    // 处理接受的数据的pub
    std::map<uint8_t, std::shared_ptr<rclcpp::Publisher<rm_message::msg::GeneralMessage>>> general_pub_;
    // 保护general_pub_的互斥锁
    std::mutex general_pub_mutex_;

    /**
     * @brief 得到对应command id 的pub
     * 
     * @param header 
     * @return std::weak_ptr<rclcpp::Subscription<rm_message::msg::GeneralMessage>>
     */
    std::weak_ptr<rclcpp::Subscription<rm_message::msg::GeneralMessage>> _get_general_pub(uint8_t header);

    // 发送数据的sub
    std::unique_ptr<rclcpp::Subscription<rm_message::msg::SendMessage>> send_sub_;

    /**
     * @brief 处理接受到的数据
     * 
     */
    void _send_sub_callback(const rm_message::msg::SendMessage::SharedPtr msg);


}; // class RMManagerNode

} // namespace RMManager

# endif // RM_MANAGER_HPP