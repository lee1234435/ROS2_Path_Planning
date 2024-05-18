import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import sys
import math
from sympy import Quaternion
from geometry_msgs.msg import Point


class go_to_goal(Node):

    def __init__(self):
        super().__init__('goal_movement_node')
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback,10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.go_to_goal_function)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.angle_to_goal =0 ;self.distance_to_goal =0
        self.robot_pose_x = 0  # Initialize robot_pose_x
        self.robot_pose_y = 0  # Initialize robot_pose_y
        self.robot_pose_z = 0  # Initialize robot_pose_z


    def pose_callback(self,data):
        self.robot_pose_x = data.pose.pose.position.x
        self.robot_pose_y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        (roll,pitch,yaw) = self.euler_from_quaternion(quaternion.x,quaternion.y,quaternion.z,quaternion.w)
        self.robot_pose.z= yaw
      
    def go_to_goal_function(self):
        self.goal_pose.x = float(sys.argv[1])
        self.goal_pose.y = float(sys.argv[2])
        self.angle_offset = float(sys.argv[3])
        # self.goal_pose.z = float(sys.argv[3])
        # self.velocity_pub.publish(msg)

        self.distance_to_goal = math.sqrt(pow((self.goal_pose.x - self.robot_pose_x),2) + pow((self.goal_pose.y - self.robot_pose_y),2))
        self.angle_to_goal = math.atan2((self.goal_pose.y - self.robot_pose_y), (self.goal_pose.x - self.robot_pose_x)) + self.angle_offset
        self.angle_to_turn = self.angle_to_goal - self.robot_pose.z
        
        if abs(self.angle_to_turn) > 0.1:
            self.vel_msg.angular.z = self.angle_to_turn
            self.vel_msg.linear.x = 0.0
        else:
            self.vel_msg.linear.x = self.distance_to_goal
            
        msg = 'Distance_To_Go : {:3f} , Angle_To_Go : {:3f}'.format(self.robot_pose_x,self.robot_pose_y,self.robot_pose_z)
        self.get_logger().info(msg)
        self.velocity_pub.publish(self.vel_msg)
        
    def euler_from_quaternion(self,x, y, z, w):
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



def main(args=None):
    rclpy.init(args=args)

    drive_publisher = go_to_goal()

    rclpy.spin(drive_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drive_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()