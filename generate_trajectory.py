#import matplotlib.pyplot as plt
#import numpy as np
#from mpl_toolkits.mplot3d import Axes3D # <--- Add this line!

# ... rest of your script

#def generate_infinity_trajectory(a, z0, num_points):
#    t = np.linspace(0, 2 * np.pi, num_points) # For one loop of Gerono lemniscate
    # For standard lemniscate, you might need a different t range or parameterization
    # e.g., t = np.linspace(-np.pi/2 + epsilon, np.pi/2 - epsilon, num_points) and similar for the other loop
    # For simplicity, let's use the Gerono lemniscate
#    x = a * np.sin(t) + x_offset
#    y = a * np.sin(t) * np.cos(t)
#    z = np.full_like(t, z0) # Constant z

#    trajectory = np.vstack((x, y, z)).T # Transpose to get (N, 3) array
#    return trajectory

#if __name__ == "__main__":
#    amplitude = 0.2 # meters, adjust based on Franka's workspace
#    height = 1.0    # meters, adjust as needed
#    x_offset = 0.4  # meters, adjust to move the entire trajectory forward from base
#    num_points = 500

#    desired_trajectory = generate_infinity_trajectory(amplitude, height, num_points)

    # Save the trajectory (e.g., to a CSV or NumPy file)
#    np.savetxt("desired_infinity_trajectory.csv", desired_trajectory, delimiter=",")

#    print(f"Generated trajectory with {num_points} points.")
#    print(f"Min/Max X: {np.min(desired_trajectory[:,0]):.3f}/{np.max(desired_trajectory[:,0]):.3f}")
#    print(f"Min/Max Y: {np.min(desired_trajectory[:,1]):.3f}/{np.max(desired_trajectory[:,1]):.3f}")
#    print(f"Z: {height:.3f}")

    # Plot for visualization
#    fig = plt.figure()
#    ax = fig.add_subplot(111, projection='3d')
#    ax.plot(desired_trajectory[:,0], desired_trajectory[:,1], desired_trajectory[:,2], label='Desired Trajectory')
#    ax.set_xlabel('X')
#    ax.set_ylabel('Y')
#    ax.set_zlabel('Z')
#    ax.set_title('Desired Infinity Trajectory')
#    ax.legend()
#    plt.show()

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def generate_infinity_trajectory(a, z0, num_points, x_offset_val=0.0): # Added x_offset_val with default
    t = np.linspace(0, 2 * np.pi, num_points)
    x = a * np.sin(t) + x_offset_val # Use the passed offset value
    y = a * np.sin(t) * np.cos(t)
    z = np.full_like(t, z0)

    trajectory = np.vstack((x, y, z)).T
    return trajectory

if __name__ == "__main__":
    amplitude = 0.2 # meters, adjust based on Franka's workspace
    height = 0.8    # meters, adjust as needed (you can try 0.9 if 0.8 is still too low)
    num_points = 500
    x_offset = 0.4  # meters, adjust to move the entire trajectory forward from base

    # Pass the x_offset to the function
    desired_trajectory = generate_infinity_trajectory(amplitude, height, num_points, x_offset_val=x_offset)

    # Save the trajectory (e.g., to a CSV or NumPy file)
    np.savetxt("desired_infinity_trajectory.csv", desired_trajectory, delimiter=",")

    print(f"Generated trajectory with {num_points} points.")
    print(f"Min/Max X: {np.min(desired_trajectory[:,0]):.3f}/{np.max(desired_trajectory[:,0]):.3f}")
    print(f"Min/Max Y: {np.min(desired_trajectory[:,1]):.3f}/{np.max(desired_trajectory[:,1]):.3f}")
    print(f"Z: {height:.3f}")

    # Optional: Plotting for visualization
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(desired_trajectory[:,0], desired_trajectory[:,1], desired_trajectory[:,2], label='Infinity Trajectory')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Desired Infinity Trajectory')
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box') # To make scales proportional
    plt.show()
