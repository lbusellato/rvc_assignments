import rclpy

from .robot import Robot

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt as e:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()