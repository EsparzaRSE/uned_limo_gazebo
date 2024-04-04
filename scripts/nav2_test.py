#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator, position_x, position_y, position_z, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = position_z
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y   
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    # -- Set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0540, 0.0)
    nav.setInitialPose(initial_pose)

    # -- Wait for Nav2
    nav.waitUntilNav2Active()

    # -- Send Nav2 goal
    goal_pose1 = create_pose_stamped(nav, 0.5857, 0.1991, 0.0540, -0.8378) 
    goal_pose2 = create_pose_stamped(nav, 1.3686, -0.6706, 0.0540, -0.1372)
    goal_pose3 = create_pose_stamped(nav, 4.6639, -0.8701, 0.0540, -0.7927)
    goal_pose4 = create_pose_stamped(nav, 4.8376, -3.7369, 0.0540, -2.2643)
    goal_pose5 = create_pose_stamped(nav, -2.3178, -3.8918, 0.0540, 2.0498)
    goal_pose6 = create_pose_stamped(nav, 0.5857, 0.1991, 0.0540, -0.8378)
    goal_pose7 = create_pose_stamped(nav, 0.5857, 0.1991, 0.0540, -0.8378)
    goal_pose8 = create_pose_stamped(nav, 0.5857, 0.1991, 0.0540, -0.8378)
    goal_pose9 = create_pose_stamped(nav, 0.5857, 0.1991, 0.0540, -0.8378)

    # --- Go to one pose
    #nav.goToPose(goal_pose1)
    #while not nav.isTaskComplete():
    #    feedback = nav.getFeedback()
        #print(feedback)
    
    # --- Follow waypoints
    waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose4, goal_pose5]
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        #print(feedback)

    print(nav.getResult())

    rclpy.shutdown()

if __name__ == '__main__':
    main()