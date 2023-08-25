import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from trajectory import Trajectory

scan_orientation = Quaternion(x=0.0017937315462992396, y=0.9999976506446824, z=0.00035651772527578343, w=-0.001163669784592465)
SCAN_START_OP = PoseStamped(pose=Pose(position=Point(x=-0.129, y=0.487, z=0.422), 
                                      orientation=scan_orientation))

planner = Trajectory()

def generate_scanning_task(initial_pose = SCAN_START_OP.pose,
                            center = np.array([0,0.487,0]),
                            displacement = 0.2,
                            rectilinear_dt = 0.5,
                            circular_dt = 1.5):
    starting_point = np.array([initial_pose.position.x, initial_pose.position.y, initial_pose.position.z])
    # First arc
    t1, u1 = planner.linear_polynomial(0, circular_dt, -np.pi/2, np.pi/2)
    q1 = planner.circular_motion_primitive(u1, starting_point, center)
    # First line
    displacement_vector = displacement * (q1[-1,:] - center) / np.linalg.norm(q1[-1,:] - center)
    t2, u2 = planner.linear_polynomial(t1[-1], t1[-1] + rectilinear_dt, 0, 1)
    q2 = planner.rectilinear_motion_primitive(u2, q1[-1,:], q1[-1,:] + displacement_vector)
    # Second arc
    t3, u3 = planner.linear_polynomial(t2[-1], t2[-1] + circular_dt, 3*np.pi/2, np.pi/2)
    q3 = planner.circular_motion_primitive(u3, q2[-1,:], center)
    # Second line
    displacement_vector = displacement * (q3[-1,:] - center) / np.linalg.norm(q3[-1,:] - center)
    t4, u4 = planner.linear_polynomial(t3[-1], t3[-1] + rectilinear_dt, 0, 1)
    q4 = planner.rectilinear_motion_primitive(u4, q3[-1,:], q3[-1,:] + displacement_vector)
    # Final arc
    _, u5 = planner.linear_polynomial(t4[-1], t4[-1] + circular_dt, -np.pi/2, np.pi/2)
    q5 = planner.circular_motion_primitive(u5, q4[-1,:], center)

    return np.concatenate((q1,q2,q3,q4,q5))

# Create a 3D plot for visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
q = generate_scanning_task()
q = q[::1]
# Plot interpolated positions
ax.plot(q[:, 0], q[:, 1], q[:, 2], label='Interpolated Positions', marker='o')

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Title and legend
plt.title('Linear Interpolation of Poses')
ax.legend()

# Show the plot
plt.show()