#include "../include/count_until_server.hpp"

rclcpp_action::GoalResponse CountUntilServerNode::goal_callback(const rclcpp_action::GoalUUID &uuid,
                                                  std::shared_ptr<const my_robot_interfaces::action::CountUntil::Goal> goal){
    (void)uuid;
    (void)goal;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CountUntilServerNode::cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUntil>> goal_handle){

    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
}

void CountUntilServerNode::handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUntil>> goal_handle){

    RCLCPP_INFO(get_logger(), "Executing the goal");
    execute_goal(goal_handle);
}

void CountUntilServerNode::execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountUntil>> goal_handle){

    // Get request from goal
    long int target_number{goal_handle->get_goal()->target_number};
    double period{goal_handle->get_goal()->period};

    // Execute the action
    int counter{0};
    rclcpp::Rate loop_rate(1.0/period);
    for(int i{0}; i < target_number; ++i){
        ++counter;
        RCLCPP_INFO(get_logger(), "%d", counter);
        loop_rate.sleep();
    }

    //Set final state and return result
    auto result{std::make_shared<CountUntil::Result>()};
    result->reached_number = counter;
    goal_handle->succeed(result);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}