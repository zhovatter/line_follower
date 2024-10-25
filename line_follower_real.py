#!/usr/bin/env python3
import math
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from pid import PID

#OBSTACLE_THRESHOLD = 0.3
#DESIRED_DIST = 0.2

class LineFollowerReal:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage,
                                          self.image_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.centroid_pub = rospy.Publisher('/line_follower/centroid/compressed', CompressedImage, queue_size=1)
        self.mask_pub = rospy.Publisher('/line_follower/mask/compressed', CompressedImage, queue_size=1)
        self.pid = PID(-0.6,0.6,0.5,0.05,2) 
        
        self.twist = Twist()
        self.image = None
        
        self.mask = None
        self.mask_msg = None

        self.centroid_msg = None
        
        self.init_states()


    def init_states(self):
        """Initialize the states of the robot."""
        self.states = {
            'following_line': True,
            'avoiding_obstacle': False,
        }

    def image_cb(self, msg): #make a mask publisher and a centroid image publisher
        """Callback to `self.image_sub`."""
        # Use the cv_bridge package to convert ROS sensor_msgs/Image messages 
        # into OpenCV2 images (cf. PRR p.197)
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # lower_yellow = numpy.array([0,  50, 10]) #too loose?
        # upper_yellow = numpy.array([255, 255, 255])

        lower_red = numpy.array([170, 70, 50])
        upper_red = numpy.array([180, 255, 255])

        self.mask = cv2.inRange(hsv, lower_red, upper_red)


    def get_centroid_center(self, image, mask):
        self.centroid_image = image.copy()
        h, w, d = image.shape #height, with, channels/data?
        search_top = math.floor(2*h/3)
        search_bot = h
        self.mask[0:search_top, 0:w] = 0
        self.mask[search_bot:h, 0:w] = 0
        M = cv2.moments(self.mask) #calculating centroid
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) #weighted x-sum of contour pixels / area of the contour
            cy = int(M['m01']/M['m00']) # weighted y-sum of contour pixels / area of the contour
            self.centroid_image = cv2.circle(self.centroid_image, (cx, cy), 20, (0,0,255), -1) #(image, center_coords, radius, color, thickness)
            self.centroid_msg = self.bridge.cv2_to_compressed_imgmsg(self.centroid_image)
            self.centroid_pub.publish(self.centroid_msg)

            self.mask_msg = self.bridge.cv2_to_compressed_imgmsg(self.mask)
            self.mask_pub.publish(self.mask_msg)

            return cx, cy
        return -1, -1 
  
    def follow_line(self):
        """Follow a red line."""
        h, w, d = self.image.shape #height, with, channels/data?
        cx, cy = self.get_centroid_center(self.image, self.mask)

        if cx >0 and cy>0:
            error = (cx - w/2) / (w/2) #extra /w/2 to reduce large numbers due to working with pixels
            print('error: ', self.pid.prev_error)
            print('integral: ', self.pid.integral)
            print('deriv: ', self.pid.derivative)
            self.twist.linear.x = 0.1
            self.twist.angular.z = self.pid.compute(0, error) #negative values mean turn left, pos means turn right
            
        else:
            self.twist.linear.x = 0
            if self.twist.angular.z == 0: 
                self.twist.angular.z = 0.3 #for searching for line initially and recovery attempt if it gets lost.
        self.cmd_vel_pub.publish(self.twist) 
        
        
    def run(self):
        """Run the program."""
        rate = rospy.Rate(10)

        for i in range(10):
            rate.sleep()

        while not rospy.is_shutdown():
            if self.states['following_line']:
                print('following_line')
                self.follow_line()
            elif self.states['avoiding_obstacle']:
                print('avoiding obstacle')
                self.avoid_obstacle()
            rate.sleep()
           
if __name__ == '__main__':
    rospy.init_node('line_follower_real')
    LineFollowerReal().run()
