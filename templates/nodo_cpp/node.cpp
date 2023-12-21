//Incluir node.hpp
#include "node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClassNameNode>(); // CAMBIAR NOMBRE
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}