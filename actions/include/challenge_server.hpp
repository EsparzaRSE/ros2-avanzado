#pragma once

#ifndef CHALLENGE_SERVER
#define CHALLENGE_SERVER

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_advanced_interfaces/action/actions_challenge.hpp"

using Challenge = my_robot_advanced_interfaces::action::ActionsChallenge;

class ChallengeServerNode : public rclcpp::Node{

    public:
        ChallengeServerNode() : Node("challenge_server"), posicion{50}{           
            challenge_server_ = rclcpp_action::create_server<Challenge>(
                this, "actions_challenge",
                std::bind(&ChallengeServerNode::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ChallengeServerNode::cancel_callback, this, std::placeholders::_1),
                std::bind(&ChallengeServerNode::handle_accepted_callback, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Action server has been started");
            RCLCPP_INFO(get_logger(), "Posición inicial del robot: %d", posicion);
        }

    private:
        int posicion{};
        rclcpp_action::Server<Challenge>::SharedPtr challenge_server_{};
        std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle_{};
        rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
                                                  std::shared_ptr<const Challenge::Goal> goal);
        rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle);
        void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle);
        void execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle);
};

#endif