#pragma once

#ifndef LIFECYCLE_CHALLENGE_SERVER_HPP
#define LIFECYCLE_CHALLENGE_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "my_robot_advanced_interfaces/action/actions_challenge.hpp"

using Challenge = my_robot_advanced_interfaces::action::ActionsChallenge;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleChallengeServerNode : public rclcpp_lifecycle::LifecycleNode{

    public:

        LifecycleChallengeServerNode() : LifecycleNode("lifecycle_challenge_server"){ 

            RCLCPP_INFO(get_logger(), "IN constructor");  
        }

        LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);
        LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);
        LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
        LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
        LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);
        LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);

    private:

        int posicion{};
        bool activation{};
        std::string nombre_accion_servidor_{};
        rclcpp_action::Server<Challenge>::SharedPtr challenge_server_{};
        std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle_{};
        rclcpp::CallbackGroup::SharedPtr cb_group_{};
        std::mutex mutex_{};
        rclcpp_action::GoalUUID preempted_goal_id_{};
        rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
                                                  std::shared_ptr<const Challenge::Goal> goal);
        rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle);
        void handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle);
        void execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle);
};

#endif