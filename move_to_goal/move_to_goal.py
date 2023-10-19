"""
This was modified from:

- P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

- Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai(@Atsushi_twi)

##################################################################
Modification for translate and rotate motion with path tracking
and ROS2 implementation was by:
Obiagba Samuel
###################################################################
"""

# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

import numpy as np

import math
import time
from threading import Thread












###############################################################################################################
# This program converts Euler angles to a quaternion.
# Author: AutomaticAddison.com
 
import numpy as np # Scientific computing library for Python
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]



# This program converts a Quaternion to Euler angles.
# Author: AutomaticAddison.com

def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """

  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)

  return roll_x, pitch_y, yaw_z # in radians

###################################################################################################################











#################################################################################
def rad_2_deg(ang_in_rad):
  ang_in_deg = ang_in_rad*180.0/math.pi
  return ang_in_deg


def deg_to_rad(ang_in_deg):
  ang_in_rad = ang_in_deg*math.pi/180
  return ang_in_rad
###################################################################################











class MoveToGoal(Node):

  def __init__(self):
    """
    Class constructor to set up the node
    """

    # Initiate the Node class's constructor and give it a name
    super().__init__('move_to_goal_node')
    
    self.posX = 0.0
    self.posY = 0.0
    self.theta = 0.0
    self.theta_inc_deg = 0.0


    self.pub_cmd = self.create_publisher(
      Twist, 
      '/cmd_vel', 
      # '/diff_drive_controller/cmd_vel_unstamped',
      10)
    self.pub_cmd # prevent unused variable warning


    self.sub_to_odom = self.create_subscription(
      Odometry, 
      '/odometry/filtered',
      # '/odom', 
      # '/diff_drive_controller/odom', 
      self.read_odometry, 
      10)
    self.sub_to_odom # prevent unused variable warning


    # self.sub_to_odom = self.create_subscription(
    #   PoseWithCovarianceStamped, 
    #   '/amcl_pose',
    #   self.read_odometry, 
    #   10)
    # self.sub_to_odom # prevent unused variable warning


    self.listen_for_pose_cmd = self.create_subscription(
      String, 
      '/move_to_goal/pose_cmd',
      self.get_pose_cmd, 
      10)
    self.listen_for_pose_cmd # prevent unused variable warning


    self.listen_for_stop_cmd = self.create_subscription(
      String, 
      '/move_to_goal/stop_cmd',
      self.get_stop_cmd, 
      10)
    self.listen_for_stop_cmd # prevent unused variable warning


    # timer_period = 0.1  # seconds
    # self.timer = self.create_timer(timer_period, self.timer_callback)


    self.old_theta_deg = rad_2_deg(self.theta)
    self.new_theta_deg = rad_2_deg(self.theta)


    self.pose_cmd_str = ""
    self.pose_cmd_arr = []
    self.goalPoseX = 0.0
    self.goalPoseY = 0.0
    self.goalPoseTheta = 0.0
    self.pose_cmd_is_available = False



    self.stop_cmd_str = ""
    self.stop_cmd_is_available = False


    # rotate and translate controller parameters ########################
    self.Kp_rho = 3.0
    self.Kp_alpha = 9.0

    self.v_max = 0.3
    self.v_min = -0.3
    

    self.Kp_beta = 5.0

    self.w_max = 1.0
    self.w_min = -1.0
    ######################################################################

    # create thread to handle actions without interrupting subscriber#####
    thread = Thread(target=self.execute_action)
    thread.daemon = True
    
    thread.start()
    #######################################################################



  def accumulate_theta_deg(self):
    self.new_theta_deg = rad_2_deg(self.theta)

    if (self.new_theta_deg - self.old_theta_deg) > 180.0:
      self.theta_inc_deg -= (self.new_theta_deg - self.old_theta_deg) - 360.0
    
    elif (self.new_theta_deg - self.old_theta_deg) < (-1 * 180.0):
      self.theta_inc_deg -= ((self.new_theta_deg - self.old_theta_deg) + 360.0)

    else:
      self.theta_inc_deg += (self.new_theta_deg - self.old_theta_deg)
  
    self.old_theta_deg = self.new_theta_deg


  
  def send_cmd(self, v, w):
    cmd_vel = Twist()
    cmd_vel.linear.x = v
    cmd_vel.linear.y = 0.000
    cmd_vel.linear.z = 0.000
    cmd_vel.angular.x = 0.000
    cmd_vel.angular.y = 0.000
    cmd_vel.angular.z = w
    self.pub_cmd.publish(cmd_vel)


  
  def read_odometry(self, odom_msg):
    self.posX = odom_msg.pose.pose.position.x
    self.posY = odom_msg.pose.pose.position.y
    
    _, _, self.theta = euler_from_quaternion(odom_msg.pose.pose.orientation.x,
                                            odom_msg.pose.pose.orientation.y,
                                            odom_msg.pose.pose.orientation.z,
                                            odom_msg.pose.pose.orientation.w)
    
    self.accumulate_theta_deg()

    print(self.posX, self.posY, self.theta)




  def get_pose_cmd(self, msg):
    self.pose_cmd_str = msg.data
    self.pose_cmd_arr = self.pose_cmd_str.split(',')
    self.goalPoseX = float(self.pose_cmd_arr[0])
    self.goalPoseY = float(self.pose_cmd_arr[1])
    self.goalPoseTheta = float(self.pose_cmd_arr[2])
    self.pose_cmd_is_available = True
    self.pose_cmd_str = ""




  def get_stop_cmd(self, msg):
    self.stop_cmd_str = msg.data
    if self.stop_cmd_str == "stop":
      self.stop_cmd_is_available = True
      self.stop_cmd_str = ""

    


   

  
  # def timer_callback(self):
  #   self.get_logger().info('posX: "%f", posY: "%f"\ntheta: "%f", theta_inc: "%f"' % (self.posX, self.posY, self.theta, self.theta_inc_deg))


  
  def rotate(self, theta_goal, accuracy):
    """
    beta is the angle between the robot's theta_pose and the goal theta_pose
    Kp_beta*beta rotates the line so that it is parallel to the goal angle
    """

    # theta = self.theta
    theta = deg_to_rad(self.theta_inc_deg)

    beta = (theta_goal - theta + np.pi) % (2 * np.pi) - np.pi

    while True:
      if (abs(beta) <= accuracy) or self.stop_cmd_is_available:
          self.send_cmd(0.0, 0.0)
          break

      beta = (theta_goal - theta + np.pi) % (2 * np.pi) - np.pi

      w = self.Kp_beta * beta

      if(w>self.w_max):
          w = self.w_max
      elif(w<self.w_min):
          w = self.w_min

      # msg = f"[beta:{round(rad_2_deg(beta),3)}, w:{round(w,3)}, theta_diff:{rad_2_deg(theta_goal - theta)}]"
      # print(msg)

      self.send_cmd(0.0, w)

      # theta = self.theta
      theta = deg_to_rad(self.theta_inc_deg)




  
  def translate(self, x_goal, y_goal, accuracy):
    """
    rho is the distance between the robot and the goal position
    alpha is the angle to the goal relative to the heading of the robot
    
    Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
    """

    x = self.posX
    y = self.posY

    # theta = self.theta
    theta = deg_to_rad(self.theta_inc_deg)

    x_diff = x_goal - x
    y_diff = y_goal - y

    rho = np.hypot(x_diff, y_diff)

    while True:
      if abs(rho) <= accuracy or self.stop_cmd_is_available:
        self.send_cmd(0.0,0.0)
        break

      x_diff = x_goal - x
      y_diff = y_goal - y

      rho = np.hypot(x_diff, y_diff)
      alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi

      v = self.Kp_rho * rho
      w = self.Kp_alpha * alpha 

      if alpha > np.pi / 2 or alpha < -np.pi / 2:
          v = -v

      if(v>self.v_max):
          v = self.v_max
      elif(v<self.v_min):
          v = self.v_min
      
      if(w>self.w_max):
          w = self.w_max
      elif(w<self.w_min):
          w = self.w_min

      # msg = f"[rho:{round(rho,3)},  x:{round(x,3)}, y:{round(y,3)}, theta:{round(rad_2_deg(theta),3)}]"
      # print(msg)

      self.send_cmd(v,w)

      x = self.posX
      y = self.posY
      
      # theta = self.theta
      theta = deg_to_rad(self.theta_inc_deg)




  def move_to_goal(self, goal_pose_array, theta_goal):
    skip = True

    for pose in goal_pose_array:    
      if skip:
         skip = False
      else:
        msg = f"CurrPose = [x:{round(self.posX,3)},  y:{round(self.posY,3)}, theta:{round(self.theta,3)}]"
        print(msg)

      theta_track = np.arctan2(pose[1]-self.posY, pose[0]-self.posX)
      self.rotate(theta_goal=theta_track, accuracy=deg_to_rad(10))
      self.translate(x_goal=pose[0], y_goal=pose[1], accuracy=0.01)

    
    self.rotate(theta_goal=theta_goal, accuracy=deg_to_rad(1))

    if self.stop_cmd_is_available:
      self.send_cmd(0.0,0.0)
      self.stop_cmd_is_available = False
      print("ERROR: Motion Stopped. Goal not Found !!!")
    
    else:
      msg = f"GoalPose = [x:{round(self.posX,3)},  y:{round(self.posY,3)}, theta:{round(self.theta,3)}]"
      print(msg)
      print("SUCCESS: Goal Found !!!")
  







############### RUN IN THREAD ####################################################
  def execute_action(self):
    time.sleep(2.0)
    pass
    # goal_pose_array = [(1.5,0), (2,2), (1.5,4), (0,4), (0,0)]
    # theta_goal = deg_to_rad(0)
    # self.move_to_goal(goal_pose_array=goal_pose_array, theta_goal=theta_goal)



    # while True:
    #   if self.pose_cmd_is_available:
    #     goal_pose_array = [(self.goalPoseX, self.goalPoseY)]
    #     theta_goal = deg_to_rad(self.goalPoseTheta)
    #     self.move_to_goal(goal_pose_array=goal_pose_array, theta_goal=theta_goal)
    #     self.pose_cmd_is_available = False  
    #   time.sleep(0.1)
#####################################################################################











def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  move_to_goal = MoveToGoal()

  # spin the node so the call back function is called
  rclpy.spin(move_to_goal)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  move_to_goal.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown() 






if __name__=='__main__':
  main()