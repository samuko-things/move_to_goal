import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SendPoseCmd(Node):

    def __init__(self):
        super().__init__('send_pose_cmd_node')
        self.command = self.create_publisher(String, '/move_to_goal/pose_cmd', 10)
        
        while rclpy.ok():
          try:
            pose_cmd_str = input('enter pose [ x(m), y(m), theta(deg) ]: ')
            pose_cmd = pose_cmd_str.split(',')

            if len(pose_cmd) < 3:
              self.get_logger().info('ERROR: incomplete pose command\n')

            # cross-check pose commands
            x = float(pose_cmd[0])
            y = float(pose_cmd[1])
            theta = float(pose_cmd[2])

            ###########################

            send_cmd = String()
            send_cmd.data = pose_cmd_str
            self.command.publish(send_cmd)
            self.get_logger().info('command-> [x=%f, y=%f, theta=%f]\n' % (x, y, theta))
          except:
            self.get_logger().info('ERROR: invalid pose commands\n')

def main(args=None):
    rclpy.init(args=args)

    send_pose_cmd = SendPoseCmd()

    rclpy.spin(send_pose_cmd)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    send_pose_cmd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()