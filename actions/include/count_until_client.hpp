#pragma once

#ifndef COUNT_UNTIL_CLIENT
#define COUNT_UNTIL_CLIENT

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_advanced_interfaces/action/count_until.hpp"

using CountUntil = my_robot_advanced_interfaces::action::CountUntil;

class CountUntilClientNode : public rclcpp::Node{

    public:
        CountUntilClientNode() : Node("count_until_client"){
            count_until_client_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
        }

        void send_goal(int target_number, double period);

    private:
        rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_{};
        void goal_response_callback(const rclcpp_action::ClientGoalHandle<CountUntil>::SharedPtr &goal_handle);
        void goal_result_callback(const rclcpp_action::ClientGoalHandle<CountUntil>::WrappedResult &result); 
        void goal_feedback_callback(const rclcpp_action::ClientGoalHandle<CountUntil>::SharedPtr &goal_handle,
                                    const std::shared_ptr<const CountUntil::Feedback> feedback); 
        // para probar si funciona al cancelar la acción 
        rclcpp::TimerBase::SharedPtr timer_{};
        rclcpp_action::ClientGoalHandle<CountUntil>::SharedPtr goal_handle_{};          
        void timer_callback();
};

#endif