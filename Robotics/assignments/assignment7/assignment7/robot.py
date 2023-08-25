import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool

from .trajectory import Trajectory

# Utility variables

RED     = 0
GREEN   = 1
BLUE    = 2
YELLOW  = 3
CUBE_NAMES =  {RED    : 'red',
               GREEN  : 'green',
               BLUE   : 'blu',
               YELLOW : 'yellow'}

# Some known waypoints, in both operational and joint space

HOME_OP = PoseStamped(pose=Pose(position=Point(x=0.484, y=0.128, z=0.609), orientation=Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)))
HOME_JOINT = np.array([0.0, -np.pi/2, np.pi/2, np.pi, -np.pi/2, -np.pi/2])

scan_orientation = Quaternion(x=0.0017937315462992396, y=0.9999976506446824, z=0.00035651772527578343, w=-0.001163669784592465)
SCAN_START_OP = PoseStamped(pose=Pose(position=Point(x=-0.129, y=0.487, z=0.422), 
                                      orientation=scan_orientation))
SCAN_START_JOINT = np.array([np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2])

class Robot(Node):

    def __init__(self):
        super().__init__('robot')

        self.cube_locations = {RED    : None,
                               GREEN  : None,
                               BLUE   : None,
                               YELLOW : None}
        
        self.cube_destinations = {RED    : None,
                                  GREEN  : None,
                                  BLUE   : None,
                                  YELLOW : None}
        # Create the publishers to the target publishing topics
        self.op_target_pub_ = self.create_publisher(PoseStamped, '/ur5/ee_target/pose', 1)
        # Create the publisher for the gripper control topic
        self.gripper_control_pub_ = self.create_publisher(Bool, '/wsg_50/controller/command', 1)
        # Create the subscribers to the pose/joint state topics
        self.op_pose_sub_ = self.create_subscription(PoseStamped, '/ur5/ee_actual/pose', self.pose_update, 1)
        self.gripper_sub_ = self.create_subscription(JointState, '/joint_states', self.gripper_state_update, 1)
        # Create the subscribers for the cube locations and destinations topics
        self.cube_subs_ = []
        for key, val in CUBE_NAMES.items():
            sub = self.create_subscription(PoseStamped, '/cube_' + val + '/pose', self.cube_callback(key, self.cube_locations), 1)
            self.cube_subs_.append(sub)
            sub = self.create_subscription(PoseStamped, '/dest_cube_' + val + '/pose', self.cube_callback(key, self.cube_destinations, 'destination'), 10)
            self.cube_subs_.append(sub)
        # These will hold the current pose and joint state of the robot
        self.pose = None
        self.joint_state = None
        # Gripper state: False for open, True for closed
        self.gripper_state = False
        # Trajectory planner
        self.planner = Trajectory()
        # Generate the scanning trajectory
        self.scanning_traj = self.generate_scanning_task()
        # Set up the control loop
        self.control_frequency = 100 #Hz
        self.dt = 1 / self.control_frequency
        self.time = 0
        self.create_timer(timer_period_sec=self.dt, callback=self.control_loop)
        # State variables
        self.homed = False
        self.reached_scan_start = False
        self.scanning_done = False

    def control_loop(self):
        if not self.homed:
            # Go home if not already there
            self.homed = self.moveL(self.pose, HOME_OP.pose, self.time)
            if self.homed: self.time = 0
        elif not self.reached_scan_start:
            # Go to the starting scanning position
            self.reached_scan_start = self.moveL(self.pose, SCAN_START_OP.pose, self.time)
            if self.reached_scan_start: self.time = 0
        elif not self.scanning_done:
            new_position, self.scanning_traj = self.pop(self.scanning_traj)
            if new_position is None:
                self.scanning_done = True
            else:
                new_pose = PoseStamped(pose=Pose(position=Point(x=new_position[0],y=new_position[1],z=new_position[2]),orientation=scan_orientation))
                self.moveTo(new_pose)
        else:
            pass

    #-------------------------------------------------------------------------------------------------------------------
    #
    #   Cartesian space movement functions
    #
    #-------------------------------------------------------------------------------------------------------------------

    def moveTo(self, target_pose):
        self.op_target_pub_.publish(target_pose)

    def moveL(self, start_pose, target_pose, t):
        if self.pose is None:
            return False
        start_pose = self.pose_to_array(start_pose)
        target_pose = self.pose_to_array(target_pose)
        # Move to a target pose by linearly interpolating from the current one.
        command_position = (1 - t) * start_pose[:3] + t * target_pose[:3]
        # SLERP on the quaternions
        command_orientation = self.slerp(start_pose[3:], target_pose[3:], self.time)
        # Send the pose command
        command_pose = PoseStamped(
            pose=Pose(position=Point(x=command_position[0],y=command_position[1],z=command_position[2]), 
                      orientation=Quaternion(x=command_orientation[0],y=command_orientation[1],z=command_orientation[2],w=command_orientation[3])))
        self.op_target_pub_.publish(command_pose)
        # We did something, update the time
        self.time += self.dt
        return np.allclose(self.pose_to_array(self.pose), target_pose, rtol=0.01)

    #-------------------------------------------------------------------------------------------------------------------
    #
    #   High level motions
    #
    #-------------------------------------------------------------------------------------------------------------------

    def generate_scanning_task(self, 
                               initial_pose = SCAN_START_OP.pose,
                               center = np.array([0,0, 0.422]),
                               displacement = 0.2,
                               rectilinear_dt = 0.5,
                               circular_dt = 1.5):
        starting_point = np.array([initial_pose.position.x, initial_pose.position.y, initial_pose.position.z])
        # First arc
        t1, u1 = self.planner.linear_polynomial(0, circular_dt, -np.pi/2, np.pi/2)
        q1 = self.planner.circular_motion_primitive(u1, starting_point, center)
        # First line
        displacement_vector = displacement * (q1[-1,:] - center) / np.linalg.norm(q1[-1,:] - center)
        t2, u2 = self.planner.linear_polynomial(t1[-1], t1[-1] + rectilinear_dt, 0, 1)
        q2 = self.planner.rectilinear_motion_primitive(u2, q1[-1,:], q1[-1,:] + displacement_vector)
        # Second arc
        t3, u3 = self.planner.linear_polynomial(t2[-1], t2[-1] + circular_dt, 3*np.pi/2, np.pi/2)
        q3 = self.planner.circular_motion_primitive(u3, q2[-1,:], center)
        # Second line
        displacement_vector = displacement * (q3[-1,:] - center) / np.linalg.norm(q3[-1,:] - center)
        t4, u4 = self.planner.linear_polynomial(t3[-1], t3[-1] + rectilinear_dt, 0, 1)
        q4 = self.planner.rectilinear_motion_primitive(u4, q3[-1,:], q3[-1,:] + displacement_vector)
        # Final arc
        _, u5 = self.planner.linear_polynomial(t4[-1], t4[-1] + circular_dt, -np.pi/2, np.pi/2)
        q5 = self.planner.circular_motion_primitive(u5, q4[-1,:], center)

        return np.concatenate((q1,q2,q3,q4,q5))

    #-------------------------------------------------------------------------------------------------------------------
    #
    #   Gripper control
    #
    #-------------------------------------------------------------------------------------------------------------------

    def close_gripper(self):
        self.gripper_control_pub_.publish(Bool(data=True))

    def open_gripper(self):
        self.gripper_control_pub_.publish(Bool(data=False))

    #-------------------------------------------------------------------------------------------------------------------
    #
    #   Callbacks for the various topics the node subscribes to
    #
    #-------------------------------------------------------------------------------------------------------------------

    def pose_update(self, data):
        self.pose = data.pose

    def gripper_state_update(self, data):
        # Don't need to record the position, just if it has closed
        self.gripper_state = np.array(data.position)[-1] < 0.0275

    def cube_callback(self, cube, cube_dict, dest=''):
        # Fills the entries in the provided dictionary, if they aren't already there
        # dest is just a cosmetic variable to distinguish between cube location and destination logging messages
        def callback(data):
            if cube_dict[cube] is None:
                cube_dict[cube] = data.pose
                position = f"Position : ({data.pose.position.x:0.3f},{data.pose.position.y:0.3f},{data.pose.position.z:0.3f})"
                orientation = f"Orientation : ({data.pose.orientation.x:0.3f},{data.pose.orientation.y:0.3f},{data.pose.orientation.z:0.3f},{data.pose.orientation.w:0.3f})"
                self.log(f'Found {CUBE_NAMES[cube].lower()} cube {dest}: {position} {orientation}')
                if not self.any_is_none(self.cube_locations) and not self.any_is_none(self.cube_destinations):
                    # All entries were filled in, change the state
                    self.scanning_done = True
        return callback
    
    #-------------------------------------------------------------------------------------------------------------------
    #
    #   Utility functions
    #
    #-------------------------------------------------------------------------------------------------------------------

    def log(self, text):
        self.get_logger().info(text)

    def any_is_none(self, dict):
        for val in dict.values():
            if val is None:
                return True
        return False

    def pose_to_array(self, pose : Pose):
        return np.array([pose.position.x, 
                        pose.position.y, 
                        pose.position.z, 
                        pose.orientation.x, 
                        pose.orientation.y, 
                        pose.orientation.z, 
                        pose.orientation.w]) if pose is not None else None
    
    def normalize(self, array):
        return array / np.linalg.norm(array)
    
    def slerp(self, q1, q2, t):
        # Compute spherical linear interpolation between quaternions
        # Ensure both input quaternions are normalized
        q1 = self.normalize(q1)
        q2 = self.normalize(q2)

        dot_product = np.dot(q1, q2)

        # Clamp the dot product to ensure it stays within [-1, 1] to avoid NaN in arccos
        dot_product = np.clip(dot_product, -1.0, 1.0)

        # Calculate the angle between the quaternions
        theta = np.arccos(dot_product)

        # Interpolate using spherical linear interpolation
        sin_theta = np.sin(theta)
        q_interpolated = (np.sin((1 - t) * theta) / sin_theta) * q1 + (np.sin(t * theta) / sin_theta) * q2

        return self.normalize(q_interpolated)
    
    def pop(self, arr):
        if arr is None:
            return None, None
        if arr.shape[0] == 1:
            return arr[0,:], None
        return arr[0,:], arr[1:,:]