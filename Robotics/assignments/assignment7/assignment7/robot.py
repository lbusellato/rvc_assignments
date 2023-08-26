import copy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .trajectory import Trajectory

class Robot(Node):

    def __init__(self):
        super().__init__('robot')

        # Utility variables

        self.RED     = 0
        self.GREEN   = 1
        self.BLUE    = 2
        self.YELLOW  = 3
        self.CUBE_NAMES =  {self.RED    : 'red',
                            self.GREEN  : 'green',
                            self.BLUE   : 'blu',
                            self.YELLOW : 'yellow'}

        # Some known waypoints

        self.HOME_OP = PoseStamped(pose=Pose(position=Point(x=0.484, y=0.128, z=0.609), orientation=Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)))
        self.SCAN_START_OP = PoseStamped(pose=Pose(position=Point(x=-0.129, y=0.487, z=0.422), orientation=Quaternion(x=0.00179, y=0.99999, z=0.00035, w=-0.00116)))

        self.cube_locations = {self.RED    : None,
                               self.GREEN  : None,
                               self.BLUE   : None,
                               self.YELLOW : None}
        
        self.cube_destinations = {self.RED    : None,
                                  self.GREEN  : None,
                                  self.BLUE   : None,
                                  self.YELLOW : None}
        
        # Create the publishers to the target publishing topics
        self.op_target_pub_ = self.create_publisher(PoseStamped, '/ur5/ee_target/pose', 1)
        # Create the publisher for the gripper control topic
        self.gripper_control_pub_ = self.create_publisher(Bool, '/wsg_50/controller/command', 1)
        # Create the subscribers to the pose/joint state topics
        self.op_pose_sub_ = self.create_subscription(PoseStamped, '/ur5/ee_actual/pose', self.pose_update, 1)
        self.gripper_sub_ = self.create_subscription(JointState, '/joint_states', self.gripper_state_update, 1)
        # Create the subscribers for the cube locations and destinations topics
        self.cube_subs_ = []
        for key, val in self.CUBE_NAMES.items():
            sub = self.create_subscription(PoseStamped, '/cube_' + val + '/pose', self.cube_callback(key, self.cube_locations), 1)
            self.cube_subs_.append(sub)
            sub = self.create_subscription(PoseStamped, '/dest_cube_' + val + '/pose', self.cube_callback(key, self.cube_destinations, 'destination'), 10)
            self.cube_subs_.append(sub)
        # These will hold the current pose and joint state of the robot
        self.pose = None
        self.joint_state = None
        # Gripper state: False for open, True for closed
        self.gripper_state = False
        # Set up the control loop
        self.control_frequency = 50 #Hz
        self.dt = 1 / self.control_frequency
        self.time = 0
        self.create_timer(timer_period_sec=self.dt, callback=self.control_loop)
        # Trajectory planner
        self.planner = Trajectory(dt=self.dt)
        # Generate the scanning trajectory
        self.scanning_traj = self.generate_scanning_task()
        # State variables
        self.homed = False
        self.reached_scan_start = False
        self.scanning_done = False

    def control_loop(self):
        if not self.scanning_done:
            if not self.scanning_traj:
                self.scanning_done = True
            else:
                self.moveTo(self.scanning_traj.pop(0))
        else:
            pass


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

    def generate_scanning_task(self, 
                               displacement = 0.2,
                               rectilinear_dt = 1.5,
                               circular_dt = 2):
        center = np.array([0.0, 0.0, self.SCAN_START_OP.pose.position.z])
        # Linear motion to the scanning start
        t, u = self.planner.linear_polynomial(0, rectilinear_dt, 0, 1)
        q = self.planner.rectilinear_motion_primitive(u, self.HOME_OP, self.SCAN_START_OP)
        # First arc
        t1, u1 = self.planner.linear_polynomial(t[-1], t[-1] + circular_dt, -np.pi/2, np.pi/2)
        q1, d1 = self.planner.circular_motion_primitive(u1, q[-1], center)
        # First line
        displacement_vector = displacement * d1
        q2f = copy.deepcopy(q1[-1])
        q2f.pose.position.x += displacement_vector[0]
        q2f.pose.position.y += displacement_vector[1]
        q2f.pose.position.z += displacement_vector[2]
        t2, u2 = self.planner.linear_polynomial(t1[-1], t1[-1] + rectilinear_dt, 0, 1)
        q2 = self.planner.rectilinear_motion_primitive(u2, q1[-1], q2f)
        # Second arc
        t3, u3 = self.planner.linear_polynomial(t2[-1], t2[-1] + circular_dt, 3*np.pi/2, np.pi/2)
        q3, d3 = self.planner.circular_motion_primitive(u3, q2[-1], center)
        # Second line
        displacement_vector = displacement * d3
        q4f = copy.deepcopy(q3[-1])
        q4f.pose.position.x += displacement_vector[0]
        q4f.pose.position.y += displacement_vector[1]
        q4f.pose.position.z += displacement_vector[2]
        t4, u4 = self.planner.linear_polynomial(t3[-1], t3[-1] + rectilinear_dt, 0, 1)
        q4 = self.planner.rectilinear_motion_primitive(u4, q3[-1], q4f)
        # Final arc
        t5, u5 = self.planner.linear_polynomial(t4[-1], t4[-1] + circular_dt, -np.pi/2, np.pi/2)
        q5, _ = self.planner.circular_motion_primitive(u5, q4[-1], center)
        # Homing
        _, u6 = self.planner.linear_polynomial(t5[-1], t5[-1] + rectilinear_dt, 0, 1)
        q6 = self.planner.rectilinear_motion_primitive(u6, q5[-1], self.HOME_OP)

        return q + q1 + q2 + q3 + q4 + q5 + q6

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
                self.log(f'Found {self.CUBE_NAMES[cube].lower()} cube {dest}: {position} {orientation}')
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