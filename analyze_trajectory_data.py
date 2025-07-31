#!/usr/bin/env python3

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import euler_from_quaternion

def analyze_trajectory_data(bag_file, desired_trajectory_file):
    
    # Load desired trajectory
    desired_trajectory = np.loadtxt(desired_trajectory_file, delimiter=",")
    print(f"Loaded desired trajectory with {len(desired_trajectory)} points.")

    executed_ee_positions = []
    executed_ee_orientations = [] # NEW: List for orientations
    executed_joint_positions = []
    executed_joint_velocities = [] # NEW: List for velocities

    print(f"Reading rosbag: {bag_file}")
    bag = rosbag.Bag(bag_file)

    # Iterate through the bag, reading from the new topics
    for topic, msg, t in bag.read_messages(topics=['/robot/executed_ee_pose', '/joint_states']):
        if topic == '/robot/executed_ee_pose':
            executed_ee_positions.append([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ])
            # NEW: Extract orientation (quaternion)
            quat = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            executed_ee_orientations.append(quat)

        elif topic == '/joint_states':
            executed_joint_positions.append(msg.position)
            # NEW: Collect velocities
            executed_joint_velocities.append(msg.velocity)

    bag.close()
    
    executed_ee_positions = np.array(executed_ee_positions)
    executed_ee_orientations = np.array(executed_ee_orientations)
    executed_joint_positions = np.array(executed_joint_positions)
    executed_joint_velocities = np.array(executed_joint_velocities)

    print(f"Collected {len(executed_ee_positions)} executed end-effector position points.")
    print(f"Collected {len(executed_joint_positions)} joint position points.")
    
    # --- Plotting ---
    
    # Figure 1: 3D Trajectory Plot (Desired vs. Executed)
    fig1 = plt.figure(figsize=(8, 6))
    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.plot(desired_trajectory[:, 0], desired_trajectory[:, 1], desired_trajectory[:, 2], 'r--', label='Desired Trajectory')
    if executed_ee_positions.shape[0] > 0:
        ax1.plot(executed_ee_positions[:, 0], executed_ee_positions[:, 1], executed_ee_positions[:, 2], 'b-', label='Executed Trajectory')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('End-Effector Trajectory: Desired vs. Executed')
    ax1.legend()
    plt.show(block=False)

    # Figure 2: Joint Positions
    if executed_joint_positions.shape[0] > 0:
        fig2 = plt.figure(figsize=(10, 6))
        ax2 = fig2.add_subplot(111)
        num_joints = executed_joint_positions.shape[1]
        for i in range(min(num_joints, 7)):
            ax2.plot(executed_joint_positions[:, i], label=f'Joint {i+1}')
        ax2.set_xlabel('Time Steps')
        ax2.set_ylabel('Joint Angle (rad)')
        ax2.set_title('Executed Joint Positions')
        ax2.legend()
        ax2.grid(True)
        plt.show(block=False)
        
    # NEW: Figure 3: End-Effector Orientation
    if executed_ee_orientations.shape[0] > 0:
        fig3 = plt.figure(figsize=(10, 6))
        ax3 = fig3.add_subplot(111)
        ee_orientations_euler = np.array([euler_from_quaternion(q) for q in executed_ee_orientations])
        ax3.plot(ee_orientations_euler[:, 0], label='Roll')
        ax3.plot(ee_orientations_euler[:, 1], label='Pitch')
        ax3.plot(ee_orientations_euler[:, 2], label='Yaw')
        ax3.set_xlabel('Time Steps')
        ax3.set_ylabel('Orientation (rad)')
        ax3.set_title('Executed End-Effector Orientation')
        ax3.legend()
        ax3.grid(True)
        plt.show(block=False)

    # NEW: Figure 4: Joint Velocities
    if executed_joint_velocities.shape[0] > 0:
        fig4 = plt.figure(figsize=(10, 6))
        ax4 = fig4.add_subplot(111)
        num_joints = executed_joint_velocities.shape[1]
        for i in range(min(num_joints, 7)):
            ax4.plot(executed_joint_velocities[:, i], label=f'Joint {i+1}')
        ax4.set_xlabel('Time Steps')
        ax4.set_ylabel('Joint Velocity (rad/s)')
        ax4.set_title('Executed Joint Velocities')
        ax4.legend()
        ax4.grid(True)
        plt.show(block=False)

    # NEW: Figure 5: Instantaneous Position Error
    if executed_ee_positions.shape[0] > 0:
        min_len = min(desired_trajectory.shape[0], executed_ee_positions.shape[0])
        position_error = np.linalg.norm(desired_trajectory[:min_len] - executed_ee_positions[:min_len], axis=1)
        
        fig5 = plt.figure(figsize=(10, 6))
        ax5 = fig5.add_subplot(111)
        ax5.plot(position_error)
        ax5.set_xlabel('Time Steps')
        ax5.set_ylabel('Position Error (m)')
        ax5.set_title('Instantaneous End-Effector Position Error')
        ax5.grid(True)
        plt.show(block=False)

    plt.show() # This final show call will block until all figures are closed.


    # --- Calculate and print errors/discrepancies ---
    if executed_ee_positions.shape[0] > 0:
        min_len = min(desired_trajectory.shape[0], executed_ee_positions.shape[0])
        position_error = np.linalg.norm(desired_trajectory[:min_len] - executed_ee_positions[:min_len], axis=1)
        print(f"\nMean End-Effector Position Error: {np.mean(position_error):.4f} m")
        print(f"Max End-Effector Position Error: {np.max(position_error):.4f} m")
    else:
        print("\nCannot calculate end-effector position error: No executed end-effector data found.")


if __name__ == '__main__':
    bag_file_path = "franka_trajectory_data_FINAL.bag"
    desired_traj_file_path = "/home/blatt/catkin_ws/src/trajectory/desired_infinity_trajectory.csv"
    
    analyze_trajectory_data(bag_file_path, desired_traj_file_path)
