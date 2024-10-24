#!/usr/bin/env python3
import math
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from pid import PID

class LineFollowerReal:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage,
                                          self.image_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pid = PID(-0.6,0.6,0.5,0.05,2) #make derivative pid a function of the callback rate? COOL IDEA: MAKE THE PROGRAM WAIT FOR A COUPLE SECONDS AND COUNT THE NUMBER OF CALLBACK CALLS TO DETERMINE THE CALLBACK RATE!  
        self.twist = Twist()
        self.image = None
        self.mask = None

        self.
        self.init_states()


    def init_states(self):
        """Initialize the states of the robot."""
        self.states = {
            'following_line': True,
            'avoiding_obstacle': False,
        }

    def my_odom_cb(self, msg):
        """Callback to `self.my_odom_sub`."""
        #raise NotImplementedError

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

        #mask = self.merge_masks(mask1, mask2)
        #raise NotImplementedError
        #raise NotImplementedError

    def get_centroid_center(self, image, mask):
        h, w, d = image.shape #height, with, channels/data?
        search_top = math.floor(2*h/3)
        search_bot = h#search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask) #calculating centroid
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) #weighted x-sum of contour pixels / area of the contour?
            #print('m00: ', M['m00'])
            #print('m10: ', M['m10'])
            cy = int(M['m01']/M['m00']) # weighted y-sum of contour pixels / area of the contour?
            return cx, cy
        return -1, -1 

    def merge_masks(self, mask1, mask2):
        new_mask = []
        for i in range(len(mask1)):
            row = []
            for j in range(len(mask1[i])):
                #print(len(mask1[i]))
                row.append(mask1[i][j] or mask2[i][j]) 
            new_mask.append(row)
        return new_mask
         

    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        
        #raise NotImplementedError
  
    def follow_line(self):
        """Follow a red line."""
        h, w, d = self.image.shape #height, with, channels/data?
        cx, cy = self.get_centroid_center(self.image, self.mask)

        if cx >0 and cy>0:
            cv2.circle(self.image, (cx, cy), 20, (150,160,140), -1) #(image, center_coords, radius, color, thickness)
            error = (cx - w/2) / (w/2) #extra /w/2 to reduce large numbers due to working with pixels?
            print('error: ', self.pid.prev_error)
            print('integral: ', self.pid.integral)
            print('deriv: ', self.pid.derivative)
            self.twist.linear.x = 0.1
            self.twist.angular.z = self.pid.compute(0, error) #negative values mean turn left, pos means turn right
            #print(self.twist.angular.z)
        else:
            self.twist.linear.x = 0
            #self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist) 
        cv2.imshow("window", mask)
        cv2.waitKey(3)
        #raise NotImplementedError

    def avoid_obstacle(self):
        """Avoid an obstacle of known dimensions."""
        raise NotImplementedError
        
    def run(self):
        """Run the program."""
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.states['following_line']:
                self.follow_line()
            elif self.states['avoiding_obstacle']:
                self.avoid_obstacle()
            rate.sleep()
           
if __name__ == '__main__':
    rospy.init_node('line_follower_real')
    LineFollowerReal().run()
