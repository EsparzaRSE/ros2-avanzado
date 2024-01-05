#pragma once

#ifndef CHALLENGE_CLIENT
#define CHALLENGE_CLIENT

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_advanced_interfaces/action/actions_challenge.hpp"

using Challenge = my_robot_advanced_interfaces::action::ActionsChallenge;

class ChallengeClientNode : public rclcpp::Node{

    public:

        ChallengeClientNode() : Node("challenge_client"){
            challenge_client_ = rclcpp_action::create_client<Challenge>(this, "actions_challenge"); 
        }

        void send_goal(int posicion, int velocidad);

    private:

        rclcpp_action::Client<Challenge>::SharedPtr challenge_client_{};
        rclcpp_action::ClientGoalHandle<Challenge>::SharedPtr goal_handle_{}; 
        void goal_response_callback(const rclcpp_action::ClientGoalHandle<Challenge>::SharedPtr &goal_handle);
        void goal_result_callback(const rclcpp_action::ClientGoalHandle<Challenge>::WrappedResult &result); 
        void goal_feedback_callback(const rclcpp_action::ClientGoalHandle<Challenge>::SharedPtr &goal_handle,
                                    const std::shared_ptr<const Challenge::Feedback> feedback);
};

#endif