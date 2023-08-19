import numpy as np
import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# Utility variables

RED     = 0
GREEN   = 1
BLUE    = 2
YELLOW  = 3
CUBE_NAMES =  {RED    : 'red',
               GREEN  : 'green',
               BLUE   : 'blu',
               YELLOW : 'yellow'}
    
HOME_CARTESIAN = Pose(position=Point(x=0.487, y=0.129, z=0.423), orientation=Quaternion(x=0.709, y=0.706, z=0.001, w=-0.001))
HOME_JOINT = JointState(position=[0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2])

class Assignment7(Node):

    def __init__(self):
        super().__init__('assignment7')

        self.cube_locations = {RED    : None,
                               GREEN  : None,
                               BLUE   : None,
                               YELLOW : None}
        
        self.cube_destinations = {RED    : None,
                                  GREEN  : None,
                                  BLUE   : None,
                                  YELLOW : None}
    
        self.cartesian_target_pub_ = self.create_publisher(PoseStamped, '/ur5/ee_target/pose', 10)
        self.cartesian_pose_sub_ = self.create_subscription(PoseStamped, '/ur5/ee_actual/pose', self.pose_update, 10)
        # Create the subscribers for the cube locations and destinations topics
        self.cube_subs_ = []
        for key, val in CUBE_NAMES.items():
            sub = self.create_subscription(PoseStamped, '/cube_' + val + '/pose', self.cube_callback(key, self.cube_locations), 10)
            self.cube_subs_.append(sub)
            sub = self.create_subscription(PoseStamped, '/dest_cube_' + val + '/pose', self.cube_callback(key, self.cube_destinations, 'destination'), 10)
            self.cube_subs_.append(sub)
        # Set the control loop frequency and start the timer
        control_frequency = 100 # Hz
        timer_period = 1 / control_frequency # seconds
        self.timer = self.create_timer(timer_period, self.controller_loop)
        # The robot starts in the home pose
        self.pose = HOME_CARTESIAN

    def controller_loop(self):
        self.cartesian_target_pub_.publish(PoseStamped(pose=HOME_CARTESIAN))
    
    def pose_update(self, data):
        self.pose = data.pose

    def cube_callback(self, cube, cube_dict, dest=''):
        # Fills the entries in the provided dictionary, if they aren't already there
        # dest is just a cosmetic variable to distinguish between cube location and destination logging messages
        def callback(data):
            if cube_dict[cube] is None:
                cube_dict[cube] = data.pose
                position = f"Position : ({data.pose.position.x:0.3f},{data.pose.position.y:0.3f},{data.pose.position.z:0.3f})"
                orientation = f"Orientation : ({data.pose.orientation.x:0.3f},{data.pose.orientation.y:0.3f},{data.pose.orientation.z:0.3f},{data.pose.orientation.w:0.3f})"
                self.get_logger().info(f'Found {CUBE_NAMES[cube].lower()} cube {dest}: {position} {orientation}')
        return callback


def main(args=None):
    rclpy.init(args=args)
    controller_node = Assignment7()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()