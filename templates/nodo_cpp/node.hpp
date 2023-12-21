#pragma once

#ifndef NODE
#define NODE

#include "rclcpp/rclcpp.hpp"

class ClassNameNode : public rclcpp::Node{  // CAMBIAR NOMBRE

    public:
        ClassNameNode() : Node("node_name"){  // CAMBIAR NOMBRE
        } 

    private: 
};

#endif