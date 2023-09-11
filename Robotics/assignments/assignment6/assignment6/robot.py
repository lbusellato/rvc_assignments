import copy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from .trajectory import Trajectory

class Robot(Node):

    def __init__(self):
        super().__init__('robot')

        # Some known waypoints
        self.HOME = PoseStamped(pose=Pose(position=Point(x=0.484, y=0.128, z=0.609), orientation=Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)))  
        self.sphere_center = np.array([0.54, 0.082, -0.018])
        # Create the publishers to the target publishing topics
        self.op_target_pub_ = self.create_publisher(PoseStamped, '/ur5/ee_target/pose', 1)
        # Create the subscribers to the sphere pose publisher topics
        self.sphere_pose_subs = []
        for i in range(3):
            self.sphere_pose_subs.append(self.create_subscription(PoseStamped, f'/sphere_{i+1}/pose', self.sphere_callback(i),1))
        # These will hold the current pose and joint state of the robot
        self.pose = None
        # Set up the control loop
        self.control_frequency = 100 #Hz
        self.dt = 1 / self.control_frequency
        self.create_timer(timer_period_sec=self.dt, callback=self.control_loop)
        # Trajectory planner
        self.planner = Trajectory(dt=self.dt)
        # Sphere positions
        self.sphere_positions = np.array([None, None, None])
        # State variables
        self.traj_generated = False
        self.traj = []

    def control_loop(self):
        if np.all(self.sphere_positions != None):
            if not self.traj_generated:
                self.log('Generating the trajectory')
                self.traj = self.generate_trajectory()
                self.traj_generated = True
            elif self.traj:
                self.moveTo(self.traj.pop(0))

    #-------------------------------------------------------------------------------------------------------------------
    #
    #   Cartesian space movement functions
    #
    #-------------------------------------------------------------------------------------------------------------------

    def moveTo(self, target_pose):
        self.op_target_pub_.publish(target_pose)

    #-------------------------------------------------------------------------------------------------------------------
    #
    #   High level motions
    #
    #-------------------------------------------------------------------------------------------------------------------    

    def generate_trajectory(self):
        # Get the data needed for traj generation
        pos1 = np.array([self.sphere_positions[0].x,self.sphere_positions[0].y,self.sphere_positions[0].z])
        pos2 = np.array([self.sphere_positions[1].x,self.sphere_positions[1].y,self.sphere_positions[1].z])
        pos3 = np.array([self.sphere_positions[2].x,self.sphere_positions[2].y,self.sphere_positions[2].z])
        # First geodesic
        q2 = self.planner.geodesic(pos1, pos2, self.sphere_center, du=0.001)
        g1_begin = q2[0]
        g1_end = q2[-1]
        # Second geodesic
        q4 = self.planner.geodesic(pos2, pos3, self.sphere_center, du=0.001)
        g2_begin = q4[0]
        g2_end = q4[-1]
        # Third geodesic
        q6 = self.planner.geodesic(pos3, pos1, self.sphere_center, du=0.001)
        g3_begin = q6[0]
        g3_end = q6[-1]
        # Linear motion to the first sphere
        _, u = self.planner.linear_polynomial(0, 2, 0, 1)
        q1 = self.planner.rectilinear_motion_primitive(u, self.HOME, g1_begin)
        # Homing
        q8 = self.planner.rectilinear_motion_primitive(u, g1_begin, self.HOME)
        # To align the orientations between the end of a geodesic and the beginning of the other
        _, u = self.planner.linear_polynomial(0, 0.5, 0, 1)
        q3 = self.planner.rectilinear_motion_primitive(u, g1_end, g2_begin)
        q5 = self.planner.rectilinear_motion_primitive(u, g2_end, g3_begin)
        q7 = self.planner.rectilinear_motion_primitive(u, g3_end, g1_begin)

        q = q1 + q2 + q3 + q4 + q5 + q6 + q7 + q8
        return q

    #-------------------------------------------------------------------------------------------------------------------
    #
    #   Callbacks for the various topics the node subscribes to
    #
    #-------------------------------------------------------------------------------------------------------------------
    
    def sphere_callback(self, sphere):
        def callback(data):
            if self.sphere_positions[sphere] is None:
                self.log(f'Adding sphere at {data.pose.position}')
                self.sphere_positions[sphere] = data.pose.position
        return callback

    #-------------------------------------------------------------------------------------------------------------------
    #
    #   Utility functions
    #
    #-------------------------------------------------------------------------------------------------------------------

    def log(self, text):
        self.get_logger().info(text)
