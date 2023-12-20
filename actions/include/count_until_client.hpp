#pragma once

#ifndef COUNT_UNTIL_CLIENT
#define COUNT_UNTIL_CLIENT

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

class CountUntilClientNode : public rclcpp::Node{

    public:
        CountUntilClientNode() : Node("count_until_client"){
        }

    private:

};

#endif