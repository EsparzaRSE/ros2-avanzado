#include "../include/lifecycle_challenge_server.hpp"

LifecycleCallbackReturn LifecycleChallengeServerNode::on_configure(const rclcpp_lifecycle::State &previous_state){

    (void)previous_state;
    RCLCPP_INFO(get_logger(), "IN on_configure");

    // parámetros por defecto, si se le pasan nuevos tomará el valor de estos últimos.
    declare_parameter("nombre_accion_servidor", "lifecycle_challenge");
    declare_parameter("posicion", 50);
    nombre_accion_servidor_ = get_parameter("nombre_servidor").as_string();
    posicion = get_parameter("posicion").as_int();

    activation = false;
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);        
    challenge_server_ = rclcpp_action::create_server<Challenge>(
                            this, nombre_accion_servidor_,
                            std::bind(&LifecycleChallengeServerNode::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
                            std::bind(&LifecycleChallengeServerNode::cancel_callback, this, std::placeholders::_1),
                            std::bind(&LifecycleChallengeServerNode::handle_accepted_callback, this, std::placeholders::_1),
                            rcl_action_server_get_default_options(),
                            cb_group_);
    RCLCPP_INFO(get_logger(), "Action server has been started");
    RCLCPP_INFO(get_logger(), "Posición inicial del robot: %d", posicion);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn LifecycleChallengeServerNode::on_cleanup(const rclcpp_lifecycle::State &previous_state){

    (void)previous_state;
    RCLCPP_INFO(get_logger(), "IN on_cleanup");
    cb_group_.reset();
    challenge_server_.reset();
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn LifecycleChallengeServerNode::on_activate(const rclcpp_lifecycle::State &previous_state){

    RCLCPP_INFO(get_logger(), "IN on_activate");
    activation = true;
    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn LifecycleChallengeServerNode::on_deactivate(const rclcpp_lifecycle::State &previous_state){

    RCLCPP_INFO(get_logger(), "IN on_deactivate");
    activation = false;
    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn LifecycleChallengeServerNode::on_shutdown(const rclcpp_lifecycle::State &previous_state){

    (void)previous_state;
    RCLCPP_INFO(get_logger(), "IN on_shutdown");
    cb_group_.reset();
    challenge_server_.reset();
    activation = false;                 
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn LifecycleChallengeServerNode::on_error(const rclcpp_lifecycle::State &previous_state){

    (void)previous_state;
    RCLCPP_INFO(get_logger(), "IN on_error");
    cb_group_.reset();
    challenge_server_.reset();
    activation = false;                    
    return LifecycleCallbackReturn::FAILURE; 
}

rclcpp_action::GoalResponse LifecycleChallengeServerNode::goal_callback(const rclcpp_action::GoalUUID &uuid,
                                                               std::shared_ptr<const Challenge::Goal> goal){
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received a goal");

    if(activation == false || (goal->posicion < 0 || goal->posicion > 100) || (goal->velocidad < 1)){

        RCLCPP_INFO(get_logger(), "Rejecting the goal");
        return rclcpp_action::GoalResponse::REJECT;
    } 

    // Policy: Preempt existing goal when receiving a new valid goal
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(goal_handle_){
            if(goal_handle_->is_active()){
                RCLCPP_INFO(get_logger(), "Abort current goal and accept new goal");
                preempted_goal_id_ = goal_handle_->get_goal_id();
                
            }
        }
    }

    RCLCPP_INFO(get_logger(), "Accepting the goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LifecycleChallengeServerNode::cancel_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle){

    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void LifecycleChallengeServerNode::handle_accepted_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle){

    RCLCPP_INFO(get_logger(), "Executing the goal");
    execute_goal(goal_handle);
}

void LifecycleChallengeServerNode::execute_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Challenge>> goal_handle){

    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }

    // Get request from goal
    int posicion_objetivo{goal_handle->get_goal()->posicion};
    int velocidad{goal_handle->get_goal()->velocidad};

    // Execute the action
    auto result{std::make_shared<Challenge::Result>()};
    auto feedback{std::make_shared<Challenge::Feedback>()};

    rclcpp::Rate loop_rate(1.0);
    while (posicion != posicion_objetivo) {

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(goal_handle->get_goal_id() == preempted_goal_id_){
                result->posicion = posicion;
                result->mensaje = "El objetivo ha sido abortado por el servidor";
                goal_handle->abort(result);
                return;
            }
        }

        if(goal_handle->is_canceling()){
            result->posicion = posicion;
            result->mensaje = "Movimiento cancelado";
            goal_handle->canceled(result);
            return;
        }
        
        if (std::abs(posicion_objetivo - posicion) < velocidad) {
            posicion = posicion_objetivo;
        }
        else{
            posicion += (posicion_objetivo > posicion ? velocidad : -velocidad);
        }

        RCLCPP_INFO(get_logger(), "Posición actual del robot: %d", posicion);
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
    auto node = std::make_shared<LifecycleChallengeServerNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}