#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import os # To check for file existence

def follow_infinity_trajectory():
    # 1. Initialize ROS and MoveIt! Commander
    print("============ Initializing moveit_commander...")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('infinity_trajectory_follower', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a MoveGroupCommander object.
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Optional: Display current state for debugging
    print("============ Planning frame: %s" % move_group.get_planning_frame())
    print("============ End effector link: %s" % move_group.get_end_effector_link())
    print("============ Robot Groups:", robot.get_group_names())
    print("============ Current robot pose:\n", move_group.get_current_pose().pose)

    # Set a slower velocity and acceleration scaling factor for smoother motion
    move_group.set_max_velocity_scaling_factor(0.6) # 60% of max velocity
    move_group.set_max_acceleration_scaling_factor(0.6) # 60% of max acceleration

    # --- NEW: Publisher for executed poses ---
    # We will publish the end-effector pose after each move to a new topic.
    executed_pose_pub = rospy.Publisher('/robot/executed_ee_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
    # Get the planning frame to use as the header's frame_id
    pose_frame_id = move_group.get_planning_frame()
    # --- END NEW ---

    # 2. Load the trajectory from CSV
    trajectory_file = os.path.join(os.environ['HOME'], 'catkin_ws', 'src', 'trajectory', 'desired_infinity_trajectory.csv')
    
    if not os.path.exists(trajectory_file):
        rospy.logerr(f"Trajectory file not found at: {trajectory_file}")
        rospy.logerr("Please ensure 'generate_trajectory.py' has been run and 'desired_infinity_trajectory.csv' exists.")
        return

    print(f"============ Loading trajectory from: {trajectory_file}")
    try:
        poses_xyz = np.loadtxt(trajectory_file, delimiter=',')
        print(f"Loaded {len(poses_xyz)} points from trajectory file.")
    except Exception as e:
        rospy.logerr(f"Error loading trajectory file: {e}")
        return

    # 3. Define a fixed orientation for the end-effector.
    target_orientation = geometry_msgs.msg.Quaternion()
    target_orientation.x = 0.0
    target_orientation.y = 0.0
    target_orientation.z = 0.0
    target_orientation.w = 1.0

    # 4. Iterate through poses and command the robot
    print("============ Starting trajectory execution...")
    for i, point_xyz in enumerate(poses_xyz):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = point_xyz[0]
        pose_goal.position.y = point_xyz[1]
        pose_goal.position.z = point_xyz[2]
        
        # Assign the fixed orientation
        pose_goal.orientation = target_orientation

        move_group.set_pose_target(pose_goal)
        
        # --- NEW: Set start state to current state to prevent planning errors ---
        # This is a critical fix to prevent the "start point deviates from current robot state" error.
        move_group.set_start_state_to_current_state()
        # --- END NEW ---

        success = move_group.go(wait=True)

        # It is always good to clear your targets after planning with poses.
        move_group.clear_pose_targets()

        if success:
            rospy.loginfo(f"Successfully moved to point {i+1}/{len(poses_xyz)}")
            # --- NEW: Publish the current EE pose after a successful move ---
            current_ee_pose = move_group.get_current_pose(move_group.get_end_effector_link()).pose
            pose_stamped_msg = geometry_msgs.msg.PoseStamped()
            pose_stamped_msg.header.stamp = rospy.Time.now()
            pose_stamped_msg.header.frame_id = pose_frame_id
            pose_stamped_msg.pose = current_ee_pose
            executed_pose_pub.publish(pose_stamped_msg)
            # --- END NEW ---
        else:
            rospy.logwarn(f"Failed to move to point {i+1}/{len(poses_xyz)}: {pose_goal}")
            break # Stop on first failure

    print("============ Trajectory execution finished.")

    # When finished, shut down MoveIt! Commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        follow_infinity_trajectory()
    except rospy.ROSInterruptException:
        pass

