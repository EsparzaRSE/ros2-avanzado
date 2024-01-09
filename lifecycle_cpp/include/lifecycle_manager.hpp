#pragma once

#ifndef LIFECYCLE_NODE_MANAGER_HPP
#define LIFECYCLE_NODE_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class LifecycleManagerNode : public rclcpp_lifecycle::LifecycleNode{

    public:

        LifecycleManagerNode() : LifecycleNode("lifecycle_manager"){         

            
        }

    private:

};

#endif