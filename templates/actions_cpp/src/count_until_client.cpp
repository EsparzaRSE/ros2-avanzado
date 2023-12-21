#include "../include/count_until_client.hpp"

void CountUntilClientNode::send_goal(int target_number, double period){

    // Wait for the Action server
    count_until_client_->wait_for_action_server();

    // Create a goal
    auto goal{CountUntil::Goal()};
    goal.target_number = target_number;
    goal.period = period;

    // Add callback
    auto options{rclcpp_action::Client<CountUntil>::SendGoalOptions()};
    options.result_callback = std::bind(&CountUntilClientNode::goal_result_callback, this, std::placeholders::_1);

    // Send the goal
    RCLCPP_INFO(get_logger(), "Sending a goal");
    count_until_client_->async_send_goal(goal, options);
}

void CountUntilClientNode::goal_result_callback(const rclcpp_action::ClientGoalHandle<CountUntil>::WrappedResult &result){

    int reached_number = result.result->reached_number;
    RCLCPP_INFO(this->get_logger(), "Result: %d", reached_number);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>();
    node->send_goal(9, 0.9);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}