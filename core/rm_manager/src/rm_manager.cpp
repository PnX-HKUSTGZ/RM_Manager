#include "rm_manager/rm_manager.hpp"
#include "rm_manager/util.hpp"

namespace RMManager {


RMManagerNode::RMManagerNode(std::string name) : Node(name) {
    // 初始化串口对象
    this->declare_parameter<std::string>("image_port", "/dev/ttyImage");
    this->declare_parameter<std::string>("referee_port", "/dev/ttyRef");
    this->declare_parameter<std::vector<std::string>>("command_topics", std::vector<std::string>{});
    this->declare_parameter<std::vector<int>>("command_code", std::vector<int>{});
    this->declare_parameter<std::string>("remote_control_topic", remote_control_topic_);

    remote_control_topic_ = this->get_parameter("remote_control_topic").as_string();
    custom_command_topics_ = this->get_parameter("command_topics").as_string_array();
    custom_command_codes_ = this->get_parameter("command_code").as_integer_array();
    std::string image_port = this->get_parameter("image_port").as_string();
    std::string referee_port = this->get_parameter("referee_port").as_string();

    // 更新 default_command_topics
    if(custom_command_topics_.size() != custom_command_codes_.size()){
        RCLCPP_ERROR(this->get_logger(), "Parameter command_topics and command_code size mismatch!");
        throw std::runtime_error("Parameter command_topics and command_code size mismatch!");
    }
    else{
        for(size_t i=0; i<custom_command_codes_.size(); i++){
            int code = custom_command_codes_[i];
            std::string topic = custom_command_topics_[i];
            if(code < 0 || code > 0xFFFF){
                RCLCPP_WARN(this->get_logger(), "Custom command code %d is out of range (0-65535), skipped.", code);
                continue;
            }
            if(topic == ""){
                RCLCPP_WARN(this->get_logger(), "Custom command topic for code %d is empty, skipped.", code);
                continue;
            }
            default_command_topics[code] = topic;
            RCLCPP_INFO(this->get_logger(), "Added custom command topic: 0x%04X -> %s", code, topic.c_str());
        }
    }

    // 验证 default_command_topics 中是否有重复的topic
    std::set<std::string> topic_set;
    for(const auto& pair : default_command_topics){
        const std::string& topic = pair.second;
        if(topic_set.find(topic) != topic_set.end()){
            RCLCPP_ERROR(this->get_logger(), "Duplicate topic name in command topics: %s", topic.c_str());
            throw std::runtime_error("Duplicate topic name in command topics: " + topic);
        }
        topic_set.insert(topic);
    }


    try{
        // 创建状态发布者
        image_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(std::string(this->get_name()) + "/image_port_status", 10);
        referee_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(std::string(this->get_name()) + "/referee_port_status", 10);
        // 创建遥控器数据发布者
        remoto_control_pub_ = this->create_publisher<rm_message::msg::RemoteControl>(std::string(this->get_name()) + remote_control_topic_, 10);

        // 基于 default_command_topics 创建对应的publisher
        for(const auto& pair : default_command_topics){
            int code = pair.first;
            const std::string& topic = pair.second;
            try{
                std::string full_topic_name = std::string(this->get_name()) + "/" + topic;
                auto pub = this->create_publisher<rm_message::msg::GeneralMessage>(full_topic_name, 10);
                general_pubs_[code] = pub;
                RCLCPP_INFO(this->get_logger(), "Created publisher for command topic: 0x%04X -> %s", code, full_topic_name.c_str());
            }
            catch(const std::exception& e){
                RCLCPP_ERROR(this->get_logger(), "Exception when create publisher for command topic 0x%04X (%s): %s", code, topic.c_str(), e.what());
            }
        }

        // 为没有有创建成功的command topic创建默认的publisher
        general_pubs_[-1] = this->create_publisher<rm_message::msg::GeneralMessage>(std::string(this->get_name()) + "/unknown_command", 10);

    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Exception when create publishers: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "Publishers initialized.");


    try{
        // 创建监测图传链路状态的线程
        if(image_port != "" && image_port != "None"){
            image_check_thread_ = std::make_unique<std::thread>([this]() {
                int nomessage_times = 0;
                rclcpp::Rate rate(1); // 1 Hz
                while (rclcpp::ok()) {
                    rate.sleep();
                    if (image_send_) {
                        nomessage_times = 0;
                        image_send_ = false;
                    } else {
                        nomessage_times++;
                    }


                    // 检查串口状态
                    if (!image_uart_->isPortOK()){
                        if(image_uart_->reopenPort() && image_uart_->startRead()){
                            RCLCPP_INFO(this->get_logger(), "Error Occurred: Image port re-opened and reading started successfully.");
                        }
                        else{
                            nomessage_times=3;
                            RCLCPP_ERROR(this->get_logger(), "Error Occurred: Image port re-open failed.");
                        }
                    }

                    std_msgs::msg::Bool status_msg;
                    if (nomessage_times >= 3) {
                        status_msg.data = false;
                        RCLCPP_WARN(this->get_logger(), "Image link seems offline!");
                    } else {
                        status_msg.data = true;
                    }
                    image_status_pub_->publish(status_msg);
                }
            });
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Exception when create image check thread: %s", e.what());
    }

    try{
        // 创建监测裁判系统链路状态的线程
        if(referee_port != "" && referee_port != "None"){
            referee_check_thread_ = std::make_unique<std::thread>([this]() {
                int nomessage_times = 0;
                rclcpp::Rate rate(1); // 1 Hz
                while (rclcpp::ok()) {
                    rate.sleep();
                    if (referee_send_) {
                        nomessage_times = 0;
                        referee_send_ = false;
                    } else {
                        nomessage_times++;
                    }

                    // 检查串口状态
                    if (!referee_uart_->isPortOK()){
                        if(referee_uart_->reopenPort() && referee_uart_->startRead()){
                            RCLCPP_INFO(this->get_logger(), "Error Occurred: Referee port re-opened and reading started successfully.");
                        }
                        else{
                            nomessage_times=3;
                            RCLCPP_ERROR(this->get_logger(), "Error Occurred: Referee port re-open failed.");
                        }
                    }

                    std_msgs::msg::Bool status_msg;
                    if (nomessage_times >= 3) {
                        status_msg.data = false;
                        RCLCPP_WARN(this->get_logger(), "Referee link seems offline!");
                    } else {
                        status_msg.data = true;
                    }
                    referee_status_pub_->publish(status_msg);
                }
            });
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Exception when create referee check thread: %s", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "Link check threads initialized.");
        
    try{

        if(image_port != ""){
            image_uart_ = std::make_shared<SerialCommunicator>(image_port, 921600);
            image_uart_->register_read_callback( std::bind(&RMManagerNode::_read_callback, this, std::placeholders::_1, std::ref(image_send_)) );
        }
        if(referee_port != ""){
            referee_uart_ = std::make_shared<SerialCommunicator>(referee_port, 921600);
            referee_uart_->register_read_callback( std::bind(&RMManagerNode::_read_callback, this, std::placeholders::_1, std::ref(referee_send_)) );
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Exception when open port: %s", e.what());
    }

    // 启动串口读取
    image_uart_->startRead();
    referee_uart_->startRead();

    RCLCPP_INFO(this->get_logger(), "Serial ports initialized and start read.");

    // 创建接受数据的sub
    send_sub_ = this->create_subscription<rm_message::msg::SendMessage>(
            "send_message", 10,
            std::bind(&RMManagerNode::_send_sub_callback, this, std::placeholders::_1)
        );

    RCLCPP_INFO(this->get_logger(), "RMManagerNode initialized.");

}

RMManagerNode::~RMManagerNode() {
    if(image_check_thread_ && image_check_thread_->joinable()){
        image_check_thread_->join();
    }
    if(referee_check_thread_ && referee_check_thread_->joinable()){
        referee_check_thread_->join();
    }
}

std::weak_ptr<rclcpp::Publisher<rm_message::msg::GeneralMessage>> RMManagerNode::_get_general_pub(uint16_t header){

    std::lock_guard<std::mutex> lock(general_pubs_mutex_);
    auto it = general_pubs_.find(header);
    if(it != general_pubs_.end()){
        return it->second;
    }
    else{
        return general_pubs_[-1];
    }

}

void RMManagerNode::_read_callback(const std::vector<uint8_t>& data, std::atomic<bool>& link_status){

    // 当前处理的待处理的数据起始位置
    std::size_t start_ptr = 0;

    while(data.size() > start_ptr){

        // 检查前两位是不是 0xA9 0x53
        // 检查有没有帧头
        if(data.size() - start_ptr < 5){
            return;
        }

        // 处理图传的特殊消息
        if(data[start_ptr] == 0xA9 && data[start_ptr + 1] == 0x53){
            if(_process_image_own_message(std::vector<uint8_t>(data.begin() + start_ptr, data.begin() + start_ptr + 21))){
                link_status = true;
            }
            start_ptr += 21;
            continue;
        }

        if(data[start_ptr] != 0xA5){
            return;
        }

        FrameHeader header;
        memcpy(&header, data.data() + start_ptr, sizeof(FrameHeader));

        // 检查 帧头 crc8
        if(Get_CRC8_Check_Sum((uint8_t*)&header, sizeof(FrameHeader)-1) != header.crc8){
            RCLCPP_WARN(this->get_logger(), "CRC8 check failed!");
            return;
        }

        // 取出数据长度并检查
        uint16_t data_length = header.data_length;
        // 计算整帧长度
        uint16_t work_load = data_length + sizeof(FrameHeader) + 4;
        if(work_load > data.size() - start_ptr){
            RCLCPP_WARN(this->get_logger(), "Data length mismatch! Expected Bigger than: %d, Actual: %zu", work_load, data.size() - start_ptr);
            return;
        }

        uint16_t command_id = *(uint16_t*)(data.data() + start_ptr + sizeof(FrameHeader));

        // 取出数据部分
        std::vector<uint8_t> payload(data.begin() + start_ptr + sizeof(FrameHeader) + 2, data.begin() + start_ptr + sizeof(FrameHeader) + 2 + data_length);

        // 取出crc16
        uint16_t received_crc = *(uint16_t*)(data.data() + start_ptr + sizeof(FrameHeader) + 2 + data_length);
        if(Get_CRC16_Check_Sum(data.data() + start_ptr, work_load-2 ) != received_crc){
            RCLCPP_WARN(this->get_logger(), "CRC16 check failed!");
            start_ptr += work_load;
            continue;
        }

        rm_message::msg::GeneralMessage msg;
        msg.cmd_id = command_id;
        msg.data_length = payload.size();
        msg.data_payload = std::move(payload);

        auto pub_weak = _get_general_pub(command_id);
        if(auto pub = pub_weak.lock()){
            pub->publish(msg);
        }

        start_ptr += work_load;
    }
}

bool RMManagerNode::_process_image_own_message(const std::vector<uint8_t>& data){

    RemoteControlData msg = {};
    memcpy(&msg, data.data(), sizeof(RemoteControlData));

    uint16_t crc = msg.crc;
    msg.crc = 0;

    // 校验CRC
    if(Get_CRC16_Check_Sum((uint8_t*)&msg, sizeof(RemoteControlData)-2) != crc){
        RCLCPP_WARN(this->get_logger(), "Image own message CRC16 check failed!");
        return false;
    }

    rm_message::msg::RemoteControl rc_msg=_remote_control_data_to_msg(msg);
    remoto_control_pub_->publish(rc_msg);
    return true;

}

rm_message::msg::RemoteControl _remote_control_data_to_msg(const RemoteControlData& data){
    rm_message::msg::RemoteControl msg;
    msg.chanal0 = data.chanal0;
    msg.chanal1 = data.chanal1;
    msg.chanal2 = data.chanal2;
    msg.chanal3 = data.chanal3;
    msg.cut = data.cut;
    msg.stop = data.stop;
    msg.keyl = data.keyl;
    msg.keyr = data.keyr;
    msg.wheel = data.wheel;
    msg.keyb = data.keyb;
    msg.mousex = data.mousex;
    msg.mousey = data.mousey;
    msg.mousez = data.mousez;
    msg.pressl = data.pressl;
    msg.pressr = data.pressr;
    msg.pressmid = data.pressmid;
    msg.w = data.keyboards.w;
    msg.s = data.keyboards.s;
    msg.a = data.keyboards.a;
    msg.d = data.keyboards.d;
    msg.shift = data.keyboards.shift;
    msg.ctrl = data.keyboards.ctrl;
    msg.q = data.keyboards.q;
    msg.e = data.keyboards.e;
    msg.r = data.keyboards.r;
    msg.f = data.keyboards.f;
    msg.g = data.keyboards.g;
    msg.z = data.keyboards.z;
    msg.x = data.keyboards.x;
    msg.c = data.keyboards.c;
    msg.v = data.keyboards.v;
    msg.b = data.keyboards.b;

    return msg;
}

void RMManagerNode::_send_sub_callback(const rm_message::msg::SendMessage::SharedPtr msg){

    // 处理帧头
    std::vector<uint8_t> frame;
    FrameHeader header = {};
    header.sof = 0xA5;
    header.data_length = msg->data_length;
    header.seq = 0;
    header.crc8 = Get_CRC8_Check_Sum((uint8_t*)&header, sizeof(FrameHeader)-1);

    frame.insert(frame.end(), (uint8_t*)&header, (uint8_t*)&header + sizeof(FrameHeader));

    // 处理命令字
    frame.push_back(msg->cmd_id & 0xFF);
    frame.push_back((msg->cmd_id >> 8) & 0xFF);

    // 处理数据部分
    frame.insert(frame.end(), msg->data_payload.begin(), msg->data_payload.end());

    // 处理crc16
    uint16_t crc16 = Get_CRC16_Check_Sum(msg->data_payload.data(), msg->data_payload.size());
    frame.push_back(crc16 & 0xFF);
    frame.push_back((crc16 >> 8) & 0xFF);

    // 发送数据
    if(msg->target == 1 && image_uart_ && image_uart_->isPortOK()){
        image_uart_->writeData(frame);
    }
    else if(msg->target == 2 && referee_uart_ && referee_uart_->isPortOK()){
        referee_uart_->writeData(frame);
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "SendMessage target error or port not open! target: %d", msg->target);
    }

}

} // namespace RMManager