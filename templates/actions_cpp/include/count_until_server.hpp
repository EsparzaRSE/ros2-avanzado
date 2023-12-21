#pragma once

#ifndef COUNT_UNTIL_SERVER
#define COUNT_UNTIL_SERVER

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;

class CountUntilServerNode : public rclcpp::Node{

    public:
        CountUntilServerNode() : Node("count_until_server"){
            count_until_server_ = rclcpp_action::create_server<CountUntil>(
                this, "count_until",
                std::bind(&CountUntilServerNode::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&CountUntilServerNode::cancel_callback, this, std::placeholders::_1),
                std::bind(&CountUntilServerNode::handle_accepted_callback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Action server has been started");
        }

    private:
        rclcpp_action::Server<CountUntil>::SharedPtr count_until_server_{};
        rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
                                                  std::shared_ptr<const CountUntil::Goal> goal);
        rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUntil>> goal_handle);
        void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUntil>> goal_handle);
        void execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUntil>> goal_handle);
};

#endif