#include "../include/count_until_server.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}