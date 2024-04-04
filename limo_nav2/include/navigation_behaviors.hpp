#pragma once

#ifndef NAVIGATION_BEHAVIORS_HPP
#define NAVIGATION_BEHAVIORS_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "yaml-cpp/yaml.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <string>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace uned_limo_gazebo {

class GoToPose : public BT::StatefulActionNode{

public:
    GoToPose(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);

    rclcpp::Node::SharedPtr node_ptr_{};
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_{};
    
    bool done_flag_{};

    // Method overrides
    static BT::PortsList providePorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};

    static BT::PortsList providedPorts();

    // Action client callback

    void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
};

} // namespace uned_limo_gazebo

#endif