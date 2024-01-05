#include "../include/challenge_server.hpp"

rclcpp_action::GoalResponse ChallengeServerNode::goal_callback(const rclcpp_action::GoalUUID &uuid,
                                                               std::shared_ptr<const Challenge::Goal> goal){

    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received a goal");

    if((goal->posicion < 0 || goal->posicion > 100) || (goal->velocidad < 1)){

        RCLCPP_INFO(get_logger(), "Rejecting the goal");
        return rclcpp_action::GoalResponse::REJECT;
    } 

    RCLCPP_INFO(get_logger(), "Accepting the goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ChallengeServerNode::cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle){

    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void ChallengeServerNode::handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle){

    RCLCPP_INFO(get_logger(), "Executing the goal");
    execute_goal(goal_handle);
}

void ChallengeServerNode::execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle){

    RCLCPP_INFO(get_logger(), "Posición actual del robot: %d", posicion);

    // Get request from goal
    int posicion_objetivo{goal_handle->get_goal()->posicion};
    int velocidad{goal_handle->get_goal()->velocidad};

    // Execute the action
    auto result{std::make_shared<Challenge::Result>()};
    auto feedback{std::make_shared<Challenge::Feedback>()};

    rclcpp::Rate loop_rate(1.0);
    while (posicion != posicion_objetivo) {
        if (std::abs(posicion_objetivo - posicion) < velocidad) {
            posicion = posicion_objetivo;
        }
        else{
            posicion += (posicion_objetivo > posicion ? velocidad : -velocidad);
        }

        feedback->posicion_actual = posicion;
        goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    //Set final state and return result
    result->posicion = posicion;
    result->mensaje = "Objetivo alcanzado";
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Posición final del robot: %d", posicion);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChallengeServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}