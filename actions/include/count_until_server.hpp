#pragma once

#ifndef COUNT_UNTIL_SERVER
#define COUNT_UNTIL_SERVER

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

class CountUntilServerNode : public rclcpp::Node{

    public:
        CountUntilServerNode() : Node("count_until_server"){
               
        }

    private:
        rclcpp_action::Server<my_robot_interfaces::action::CountUntil>::SharedPtr count_until_server_{};
};

#endif