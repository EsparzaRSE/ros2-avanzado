#pragma once

#ifndef CHALLENGE_CLIENT
#define CHALLENGE_CLIENT

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_advanced_interfaces/action/actions_challenge.hpp"
#include "my_robot_advanced_interfaces/msg/cancel_move_challenge.hpp"

using Challenge = my_robot_advanced_interfaces::action::ActionsChallenge;

class ChallengeClientNode : public rclcpp::Node{

    public:

        ChallengeClientNode() : Node("challenge_client"){
            subscriber_cancel_move_ = create_subscription<my_robot_advanced_interfaces::msg::CancelMoveChallenge>
                                      ("cancel_move", 10, std::bind(&ChallengeClientNode::send_cancel_goal, this, std::placeholders::_1));
            challenge_client_ = rclcpp_action::create_client<Challenge>(this, "actions_challenge"); 
        }

        void send_goal(int posicion, int velocidad);

    private:

        rclcpp_action::Client<Challenge>::SharedPtr challenge_client_{};
        rclcpp_action::ClientGoalHandle<Challenge>::SharedPtr goal_handle_{}; 
        rclcpp::Subscription<my_robot_advanced_interfaces::msg::CancelMoveChallenge>::SharedPtr subscriber_cancel_move_{};
        void goal_response_callback(const rclcpp_action::ClientGoalHandle<Challenge>::SharedPtr &goal_handle);
        void goal_result_callback(const rclcpp_action::ClientGoalHandle<Challenge>::WrappedResult &result); 
        void goal_feedback_callback(const rclcpp_action::ClientGoalHandle<Challenge>::SharedPtr &goal_handle,
                                    const std::shared_ptr<const Challenge::Feedback> feedback);
        void send_cancel_goal(const my_robot_advanced_interfaces::msg::CancelMoveChallenge msg);
};

#endif