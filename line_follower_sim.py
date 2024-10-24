#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import math
from pid import PID

class LineFollowerSim:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw/compressed',
                                          CompressedImage,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.pid = PID(-0.6,0.6,0.2,0.05,2) #make derivative pid a function of the callback rate? COOL IDEA: MAKE THE PROGRAM WAIT FOR A COUPLE SECONDS AND COUNT THE NUMBER OF CALLBACK CALLS TO DETERMINE THE CALLBACK RATE!  

    def image_callback(self, msg):
        """Callback to `self.image_sub`."""
        #print(len(msg.data))
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([0,  50, 10]) #too loose?
        upper_yellow = numpy.array([255, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)


        h, w, d = image.shape #height, with, channels/data?
        cx, cy = self.get_centroid_center(image, mask)

        if cx >0 and cy>0:
            cv2.circle(image, (cx, cy), 20, (150,160,140), -1) #(image, center_coords, radius, color, thickness)
            error = (cx - w/2) / (w/2) #extra /w/2 to reduce large numbers due to working with pixels?
            print('error: ', self.pid.prev_error)
            print('integral: ', self.pid.integral)
            print('deriv: ', self.pid.derivative)
            self.twist.linear.x = 0.2
            self.twist.angular.z = self.pid.compute(0, error) #negative values mean turn left, pos means turn right
            #print(self.twist.angular.z)
            self.cmd_vel_pub.publish(self.twist) 
        #cv2.imshow("window", mask)
        #cv2.waitKey(3)
        #raise NotImplementedError

    def get_centroid_center(self, image, mask):
        h, w, d = image.shape #height, with, channels/data?
        search_top = math.floor(3*h/4)
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask) #calculating centroid
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) #weighted x-sum of contour pixels / area of the contour?
            #print('m00: ', M['m00'])
            #print('m10: ', M['m10'])
            cy = int(M['m01']/M['m00']) # weighted y-sum of contour pixels / area of the contour?
            return cx, cy
        return -1, -1 #
         

     
    
    def follow_line(self):
        """Follow a yellow line."""
        #raise NotImplementedError

    def run(self):
        """Run the Program."""
        rate = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            self.follow_line()

if __name__ == '__main__':
    rospy.init_node('line_follower_sim')
    LineFollowerSim().run()
