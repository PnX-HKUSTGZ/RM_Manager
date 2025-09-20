#include "rm_manager/rm_manager.hpp"
#include "rm_manager/util.hpp"

namespace RMManager {


RMManagerNode::RMManagerNode(std::string name) : Node(name) {
    // 初始化串口对象
    this->declare_parameter<std::string>("image_port", "/dev/ttyImage");
    this->declare_parameter<std::string>("referee_port", "/dev/ttyRef");
    std::string image_port = this->get_parameter("image_port").as_string();
    std::string referee_port = this->get_parameter("referee_port").as_string();
    try{
        // 创建状态发布者
        image_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(std::string(this->get_name()) + "/image_port_status", 10);
        referee_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(std::string(this->get_name()) + "/referee_port_status", 10);
        // 创建遥控器数据发布者
        remoto_controll_pub_ = this->create_publisher<rm_message::msg::RemotoControll>(std::string(this->get_name()) + "/remoto_controll", 10);
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
        // 创建新的publisher
        std::string topic_name = std::string(this->get_name()) + "/" + uint16_to_hex_string_with_prefix(header);
        auto pub = this->create_publisher<rm_message::msg::GeneralMessage>(topic_name, 10);
        general_pubs_[header] = pub;
        RCLCPP_INFO(this->get_logger(), "Created new publisher for topic: %s", topic_name.c_str());
        return pub;
    }

}

void RMManagerNode::_read_callback(const std::vector<uint8_t>& data, std::atomic<bool>& link_status){

    // 检查前两位是不是 0xA9 0x53
    if(data.size() < 5){
        return;
    }

    if(data[0] == 0xA9 && data[1] == 0x53){
        if(_process_image_own_message(data)){
            link_status = true;
        }
        return;
    }

    if(data[0] != 0xA5){
        return;
    }

    FrameHeader header;
    memcpy(&header, data.data(), sizeof(FrameHeader));

    // 检查 帧头 crc8
    if(Get_CRC8_Check_Sum((uint8_t*)&header, sizeof(FrameHeader)-1) != header.crc8){
        RCLCPP_WARN(this->get_logger(), "CRC8 check failed!");
        return;
    }

    // 取出数据长度并检查
    uint16_t data_length = header.data_length;
    if(data_length + sizeof(FrameHeader) + 4 != data.size()){
        RCLCPP_WARN(this->get_logger(), "Data length mismatch! Expected: %ld, Actual: %zu", data_length + sizeof(FrameHeader) + 4, data.size());
        return;
    }

    uint16_t command_id = *(uint16_t*)(data.data() + sizeof(FrameHeader));

    // 取出数据部分
    std::vector<uint8_t> payload(data.begin() + sizeof(FrameHeader) + 2, data.begin() + sizeof(FrameHeader) + 2 + data_length);

    // 取出crc16
    uint16_t received_crc = *(uint16_t*)(data.data() + sizeof(FrameHeader) + 2 + data_length);
    if(Get_CRC16_Check_Sum(data.data(), data.size()-2 ) != received_crc){
        RCLCPP_WARN(this->get_logger(), "CRC16 check failed!");
        return;
    }

    rm_message::msg::GeneralMessage msg;
    msg.cmd_id = command_id;
    msg.data_length = payload.size();
    msg.data_payload = std::move(payload);

    auto pub_weak = _get_general_pub(command_id);
    if(auto pub = pub_weak.lock()){
        pub->publish(msg);
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

    rm_message::msg::RemotoControll rc_msg=_remote_control_data_to_msg(msg);
    remoto_controll_pub_->publish(rc_msg);
    return true;

}

rm_message::msg::RemotoControll _remote_control_data_to_msg(const RemoteControlData& data){
    rm_message::msg::RemotoControll msg;
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