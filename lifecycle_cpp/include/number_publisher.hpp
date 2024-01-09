#pragma once

#ifndef NUMBER_PUBLISHER
#define NUMBER_PUBLISHER

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "example_interfaces/msg/int64.hpp"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NumberPublisherNode : public rclcpp_lifecycle::LifecycleNode{

    public:

        NumberPublisherNode() : LifecycleNode("number_publisher"), number_(1), publish_frequency_(1.0){         

            RCLCPP_INFO(get_logger(), "IN constructor");
        }

        LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);
        LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);
        LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
        LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
        LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);
        LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);

    private:

        int number_{};
        double publish_frequency_{};
        rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_{};
        rclcpp::TimerBase::SharedPtr number_timer_{};
        void publishNumber();
};

#endif