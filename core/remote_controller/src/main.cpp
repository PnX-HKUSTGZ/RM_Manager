#include "remote_controller/remote_controller.hpp"

int main (int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RM_REMOTE_CONTROLLER::RemoteController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}