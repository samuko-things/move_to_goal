import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from pynput.keyboard import Key, Listener



class StopMotion(Node):
  def __init__(self):
    super().__init__(node_name="stop_motion_node") # initialize the node name

    self.send_stop_cmd = self.create_publisher(String, 'move_to_goal/stop_cmd', 10)
    
    # ...or, in a non-blocking fashion:
    listener = Listener(on_press=self.on_press, on_release=self.on_release)
    listener.start()
    # listener.join()
            

  def publish_stop_cmd(self):
    stop_cmd = String()
    stop_cmd.data = "stop"
    self.send_stop_cmd.publish(stop_cmd)
    print("motion stopped")



  def on_press(self, key):       
    pass
            
  def on_release(self, key):
    if key == Key.esc:
      self.publish_stop_cmd()
        






def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  stop_motion = StopMotion()

  # spin the node so the call back function is called
  rclpy.spin(stop_motion)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  stop_motion.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown() 



if __name__=='__main__':
  main()