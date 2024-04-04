#pragma once

#ifndef CIRCUITO_VUELTA_HPP
#define CIRCUITO_VUELTA_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigation_behaviors.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace uned_limo_gazebo {

class CircuitoVuelta : public rclcpp::Node{

public:
    explicit CircuitoVuelta(const rclcpp::NodeOptions &options);

    void setup();
    void create_behavior_tree();
    void update_behavior_tree();
    

private:
    rclcpp::TimerBase::SharedPtr timer_{};
    rclcpp::TimerBase::SharedPtr setup_timer_{};
    BT::Tree tree_{};
};

} // namespace uned_limo_gazebo

#endif