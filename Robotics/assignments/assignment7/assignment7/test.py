import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

def linear_interpolation(start_pose, target_pose, t):
    # Linear interpolation for position
    command_position = (1 - t) * np.array(start_pose[:3]) + t * np.array(target_pose[:3])

    # Spherical linear interpolation (SLERP) for quaternion
    theta = np.arccos(np.clip(np.dot(start_pose[3:], target_pose[3:]), -1, 1))
    command_orientation = (np.sin((1 - t) * theta) / np.sin(theta)) * np.array(start_pose[3:]) + (np.sin(t * theta) / np.sin(theta)) * np.array(target_pose[3:])

    return command_position, command_orientation

# Example start and target poses
start_pose = [0.484, 0.128, 0.609, 0.5, 0.5, 0.5, 0.5]  # [x, y, z, qx, qy, qz, qw]
target_pose = [-0.129, 0.487, 0.422, 0.0017937315462992396, 0.9999976506446824, 0.00035651772527578343, -0.001163669784592465]

# Number of steps for interpolation
num_steps = 50

# Create arrays to store interpolated poses
interpolated_positions = []
interpolated_orientations = []

# Perform interpolation and store results
for step in range(num_steps):
    t = step / (num_steps - 1)  # t varies from 0 to 1
    position, orientation = linear_interpolation(start_pose, target_pose, t)
    interpolated_positions.append(position)
    interpolated_orientations.append(orientation)

# Convert lists to arrays
interpolated_positions = np.array(interpolated_positions)
interpolated_orientations = np.array(interpolated_orientations)

# Create a 3D plot for visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot interpolated positions
ax.plot(interpolated_positions[:, 0], interpolated_positions[:, 1], interpolated_positions[:, 2], label='Interpolated Positions', marker='o')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Title and legend
plt.title('Linear Interpolation of Poses')
ax.legend()

# Show the plot
plt.show()