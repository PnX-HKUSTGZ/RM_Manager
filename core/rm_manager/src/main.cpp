#include "rm_manager/rm_manager.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RMManager::RMManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}