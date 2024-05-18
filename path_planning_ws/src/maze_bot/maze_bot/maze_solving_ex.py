# --------------- prarice scripts --------------- #
from geometry_msgs.msg import Twist
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import os
import rclpy
import cv2

# --------------- main scripts --------------- # 
class Video_get(Node):
    def __init__(self):
        super().__init__('maze_solving_ex') # node name       
        # create a subscriber
        self.subscriber = self.create_subscription(Image, 'upper_ex_camera/Image_raw',self.get_video_feed_cb,10)
        
        # create a publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.maze_solving)
        
    def get_video_feed_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion
        self.out.write(frame) # write the frames to video
        cv2.imshow('output',frame) # displaying what is being recorded
        cv2.waitKey(1) # will save video until it is interrupted
        
    def maze_solving(self):
        msg = Twist()
        msg.linear.x=0.5
        msg.angular.z=0.3 
        
        self.publisher_.publish(msg)

        # setting for writing the frames into a video
        vid_path = os.path.join(os.getcwd(),'output.avi')
        self.out = cv2.VideoWriter(vid_path,cv2.VideoWriter_fourcc('M','J','P','G'),30,(1280,720))
        self.bridge = CvBridge() # converting ros image to opencv data
        

        
# --------------- execute scripts --------------- # 
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = Video_get()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
        
