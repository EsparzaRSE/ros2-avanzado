#include "../include/number_publisher.hpp"

LifecycleCallbackReturn NumberPublisherNode::on_configure(const rclcpp_lifecycle::State &previous_state){

    (void)previous_state;
    RCLCPP_INFO(get_logger(), "IN on_configure");
    number_publisher_ = create_publisher<example_interfaces::msg::Int64>("number", 10);
    number_timer_ = create_wall_timer(std::chrono::milliseconds((int)(1000.0 / publish_frequency_)),
                                      std::bind(&NumberPublisherNode::publishNumber, this));
    number_timer_->cancel();
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn NumberPublisherNode::on_cleanup(const rclcpp_lifecycle::State &previous_state){

    (void)previous_state;
    RCLCPP_INFO(get_logger(), "IN on_cleanup");
    number_publisher_.reset();
    number_timer_.reset();                         //Con el punto reseteas el puntero
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn NumberPublisherNode::on_activate(const rclcpp_lifecycle::State &previous_state){

    RCLCPP_INFO(get_logger(), "IN on_activate");
    number_timer_->reset();                        //Con la flecha reseteas el timer en sí
    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;

}

LifecycleCallbackReturn NumberPublisherNode::on_deactivate(const rclcpp_lifecycle::State &previous_state){

    RCLCPP_INFO(get_logger(), "IN on_deactivate");
    number_timer_->cancel();
    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn NumberPublisherNode::on_shutdown(const rclcpp_lifecycle::State &previous_state){

    (void)previous_state;
    RCLCPP_INFO(get_logger(), "IN on_shutdown");
    number_publisher_.reset();
    number_timer_.reset();                   
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn NumberPublisherNode::on_error(const rclcpp_lifecycle::State &previous_state){

    (void)previous_state;
    RCLCPP_INFO(get_logger(), "IN on_error");
    number_publisher_.reset();
    number_timer_.reset();                   
    return LifecycleCallbackReturn::FAILURE; 
}

void NumberPublisherNode::publishNumber(){
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    number_publisher_->publish(msg);
    number_++;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
