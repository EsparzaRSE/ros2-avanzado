#include "../include/challenge_client.hpp"

void ChallengeClientNode::send_goal(int posicion, int velocidad){

    // Wait for the Action server
    challenge_client_->wait_for_action_server();

    // Create a goal
    auto goal{Challenge::Goal()};
    goal.posicion = posicion;
    goal.velocidad = velocidad;

    // Add callback
    auto options{rclcpp_action::Client<Challenge>::SendGoalOptions()};
    options.goal_response_callback = std::bind(&ChallengeClientNode::goal_response_callback, this, std::placeholders::_1);
    options.result_callback = std::bind(&ChallengeClientNode::goal_result_callback, this, std::placeholders::_1);
    options.feedback_callback = std::bind(&ChallengeClientNode::goal_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    // Send the goal
    RCLCPP_INFO(get_logger(), "Enviado el objetivo con posición: %d y velocidad %d", posicion, velocidad);
    challenge_client_->async_send_goal(goal, options);
}

void ChallengeClientNode::goal_response_callback(const rclcpp_action::ClientGoalHandle<Challenge>::SharedPtr &goal_handle){

    if(!goal_handle){
        RCLCPP_INFO(get_logger(), "Goal got rejected");
    }
    else{
        this->goal_handle_ = goal_handle;
        RCLCPP_INFO(get_logger(), "Goal got accepted");  
    }
}

void ChallengeClientNode::goal_result_callback(const rclcpp_action::ClientGoalHandle<Challenge>::WrappedResult &result){

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

    int posicion_resultado{result.result->posicion};
    std::string mensaje{result.result->mensaje};

    RCLCPP_INFO(get_logger(), "Posición: %d", posicion_resultado);
    RCLCPP_INFO(get_logger(), "%s", mensaje.c_str());
}


void ChallengeClientNode::goal_feedback_callback(const rclcpp_action::ClientGoalHandle<Challenge>::SharedPtr &goal_handle,
                                                 const std::shared_ptr<const Challenge::Feedback> feedback){

    (void)goal_handle;
    int posicion_actual{feedback->posicion_actual};
    RCLCPP_INFO(get_logger(), "Got feedback: %d", posicion_actual);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChallengeClientNode>();
    node->send_goal(21, 9);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}