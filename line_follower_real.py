#!/usr/bin/env python3
import math
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from pid import PID

OBSTACLE_THRESHOLD = 0.2
DESIRED_DIST = 0.2

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
        self.centroid_pub = rospy.Publisher('/line_follower/centroid/compressed', CompressedImage, queue_size=1)
        self.mask_pub = rospy.Publisher('/line_follower/mask/compressed', CompressedImage, queue_size=1)
        self.pid = PID(-0.6,0.6,0.5,0.05,2) #make derivative pid a function of the callback rate? COOL IDEA: MAKE THE PROGRAM WAIT FOR A COUPLE SECONDS AND COUNT THE NUMBER OF CALLBACK CALLS TO DETERMINE THE CALLBACK RATE! 
        self.wall_pid = PID(min_val = -0.6, max_val = 0.6, kp = 1, ki = 0.05, kd = 40) #A high derivative term is used to to the small values of each derivative. 
        self.twist = Twist()
        self.image = None
        self.mask = None
        self.distFromWall = 0
        self.lidar = None

        self.cur_yaw = None
        self.init_states()


    def init_states(self):
        """Initialize the states of the robot."""
        self.states = {
            'following_line': True,
            'avoiding_obstacle': False,
        }

    def my_odom_cb(self, msg):
        """Callback to `self.my_odom_sub`."""
        self.cur_yaw = msg.y
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
        self.centroid_image = image.copy()
        h, w, d = image.shape #height, with, channels/data?
        search_top = math.floor(2*h/3)
        search_bot = h#search_top + 20
        self.mask[0:search_top, 0:w] = 0
        self.mask[search_bot:h, 0:w] = 0
        M = cv2.moments(self.mask) #calculating centroid
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) #weighted x-sum of contour pixels / area of the contour?
            #print('m00: ', M['m00'])
            #print('m10: ', M['m10'])
            cy = int(M['m01']/M['m00']) # weighted y-sum of contour pixels / area of the contour?
            self.centroid_image = cv2.circle(self.centroid_image, (cx, cy), 20, (0,0,255), -1) #(image, center_coords, radius, color, thickness)
            #self.centroid_image = self.bridge.cv2_to_compressed_imgmsg(self.centroid_image)

            #self.centroid_pub.publish(self.centroid_image)
            #self.mask = self.bridge.cv2_to_compressed_imgmsg(self.mask)
            #self.mask_pub.publish(self.mask)
            return cx, cy
        #self.centroid_pub.publish(self.centroid_image)
        #self.mask = self.bridge.cv2_to_compressed_imgmsg(self.mask)
        #self.mask_pub.publish(self.mask)
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
        #print(msg)
        self.lidar = self.cleanLidar(list(msg.ranges))
        #print(self.lidar)
        wallLidar = self.lidar[210:359]
        #print(wallLidar)
        wallLidar.extend(self.lidar[0:9])

        self.rightLidar = self.lidar[180:359]
        self.distFromWall = self.dist_to_wall(wallLidar)
        if self.distFromWall < OBSTACLE_THRESHOLD:
            self.cmd_vel_pub.publish(Twist()) #stops pid commands from line following
            self.states['avoiding_obstacle'] == True

        #raise NotImplementedError

    def cleanLidar(self, lidar):
        """Cleans lidar values that are outside of valid range"""
        for i in range(len(lidar)):
            if lidar[i] < 0.12 or lidar[i] > 3.5:
                lidar[i] = 0
        return lidar

    def dist_to_wall(self, lidar):
        """Calculates distance to the wall by finding the minimum lidar value in the range"""
        min = 3.5 #3.5 is the max lidar value
        for i in range(len(lidar)):
            if lidar[i] != 0 and lidar[i] < min:
                min = lidar[i]
        return min
  
    def follow_line(self):
        """Follow a red line."""
        h, w, d = self.image.shape #height, with, channels/data?
        cx, cy = self.get_centroid_center(self.image, self.mask)

        if cx >0 and cy>0:
            self.states['avoiding_obstacle'] == False
            error = (cx - w/2) / (w/2) #extra /w/2 to reduce large numbers due to working with pixels?
            print('error: ', self.pid.prev_error)
            print('integral: ', self.pid.integral)
            print('deriv: ', self.pid.derivative)
            self.twist.linear.x = 0.1
            self.twist.angular.z = self.pid.compute(0, error) #negative values mean turn left, pos means turn right
            #print(self.twist.angular.z)
        else:
            self.twist.linear.x = 0
            if self.twist.angular.z == 0 and self.states['avoiding_obstacle'] == False:
                self.twist.angular.z = 0.1 #for searching for line initially and recovery attempt if it gets lost.
            #self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist) 
       #cv2.imshow("window", mask)
        #cv2.waitKey(3)
        #raise NotImplementedError

    def avoid_obstacle(self):
        """Avoid an obstacle of known dimensions."""
        self.states['following_line'] == False
        self.twist.linear.x = 0.1
        self.turn_to_heading(0.5*math.pi)
        self.twist.angular.z = self.wall_pid.compute(setpoint = DESIRED_DIST, measured_value = self.distFromWall)
        self.cmd_vel_pub.publish(self.twist)

        #raise NotImplementedError
    
    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        headingTwist = Twist()
        proportion = 0.5

        while not math.isclose(self.cur_yaw, target_yaw, abs_tol=TURN_ANGLE_TOLERANCE): #since going past pi rads turns heading value to negative
            #slows rotation as the target is approached, with a minimum and maximum rotation speed defined.
            headingTwist.angular.z = max(min(abs(proportion*(target_yaw - self.cur_yaw)) , MAX_TURN_SPEED) , MIN_TURN_SPEED) 
            self.cmd_vel_pub.publish(headingTwist)

            while self.cur_yaw == None:
                continue #waiting for new odometry values
       
        self.cmd_vel_pub.publish(Twist()) #a 0-rotation twist to stop movement

         #converts negative angles (past 1pi radians) to a positive value
    def convertNegativeAngles(self, angle):
        if angle >= 0:
            return angle
        else:
            return 2*math.pi + angle 
    
    #makes sure there are no angles over 2pi
    def normalizePosAngles(self, heading):
        return abs(heading) % (2*math.pi)

    #ensures yaw targets are relative to the robot, allowing it to move in the desired shapes from any initial orientation.
    def headingToRelative(self, target_yaw):
        return self.normalizePosAngles(self.cur_yaw + target_yaw)
        
    def run(self):
        """Run the program."""
        rate = rospy.Rate(10)
        #while self.image == None or self.cur_yaw == None or self.lidar == None:
        for i in range(10):
            rate.sleep()

        while not rospy.is_shutdown():
            if self.states['following_line']:
                self.follow_line()
            elif self.states['avoiding_obstacle']:
                self.avoid_obstacle()
            rate.sleep()
           
if __name__ == '__main__':
    rospy.init_node('line_follower_real')
    LineFollowerReal().run()
