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
    options.goal_response_callback = std::bind(&CountUntilClientNode::goal_response_callback, this, std::placeholders::_1);
    options.result_callback = std::bind(&CountUntilClientNode::goal_result_callback, this, std::placeholders::_1);
    options.feedback_callback = std::bind(&CountUntilClientNode::goal_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    // Send the goal
    RCLCPP_INFO(get_logger(), "Sending a goal");
    count_until_client_->async_send_goal(goal, options);
    timer_ = create_wall_timer(std::chrono::seconds(2), std::bind(&CountUntilClientNode::timer_callback, this));
}

void CountUntilClientNode::goal_response_callback(const rclcpp_action::ClientGoalHandle<CountUntil>::SharedPtr &goal_handle){

    if(!goal_handle){
        RCLCPP_INFO(get_logger(), "Goal got rejected");
    }
    else{
        this->goal_handle_ = goal_handle;
        RCLCPP_INFO(get_logger(), "Goal got accepted");  
    }
}

void CountUntilClientNode::goal_result_callback(const rclcpp_action::ClientGoalHandle<CountUntil>::WrappedResult &result){

    auto status{result.code};

    if(status == rclcpp_action::ResultCode::SUCCEEDED){
        RCLCPP_INFO(get_logger(), "SUCCEEDED"); 
    }
    else if(status == rclcpp_action::ResultCode::ABORTED){
        RCLCPP_ERROR(get_logger(), "ABORTED"); 
    }
    else if(status == rclcpp_action::ResultCode::CANCELED){
        RCLCPP_WARN(get_logger(), "CANCELED"); 
    }

    int reached_number = result.result->reached_number;
    RCLCPP_INFO(this->get_logger(), "Result: %d", reached_number);
}

void CountUntilClientNode::goal_feedback_callback(const rclcpp_action::ClientGoalHandle<CountUntil>::SharedPtr &goal_handle,
                                    const std::shared_ptr<const CountUntil::Feedback> feedback){
    
    (void)goal_handle;
    long int number{feedback->current_number};
    RCLCPP_INFO(get_logger(), "Got feedback: %ld", number);
}

// para probar si funciona al cancelar
void CountUntilClientNode::timer_callback(){

    RCLCPP_INFO(get_logger(), "Cancel the goal");
    count_until_client_->async_cancel_goal(goal_handle_);
    timer_->cancel();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>();
    node->send_goal(8, 0.9);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}