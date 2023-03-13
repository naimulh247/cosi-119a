#!/usr/bin/env python 
 
import rospy 
import cv2, cv_bridge
import numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class Follower:
    def __init__(self) -> None:
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.last_err = 0
        
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # bounds for yellow
        lower_yellow = numpy.array([ 50,  50, 170])
        upper_yellow = numpy.array([120, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        

        h, w, d = image.shape
        # limit the search area to the bottom half of the image
        search_top = int(3*h/4)
        search_bot = int(3*h/4) + 30
        # limit the left and right window so that it doesn't go to a nearby line
        search_left = int(w/2) - int(w/4)
        search_right = int(w/2) + int(w/4)       
        # mask it off 
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        mask[0:h, 0:search_left] = 0
        mask[0:h, search_right:w] = 0

        M = cv2.moments(mask)
        if M['m00'] > 0:
            
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

            # Begin PID control
            err = cx - w/2
            k_p = 0.01  # Proportional gain
            max_linear_x = 0.2  # Maximum linear velocity
            min_linear_x = 0.05  # Minimum linear velocity
            self.twist.angular.z = -float(err) / 100
            self.last_err = self.twist.angular.z
            self.twist.linear.x = max(min_linear_x, max_linear_x - k_p * abs(err))
            # End PID control
        # if the line is lost, rotate the last angle
        elif self.last_err != 0:
            
            self.twist.linear.x = 0.0
            self.twist.angular.z = self.last_err
        
        # if there was no last angle, or no line was detected, search the entire view for a line and go there
        else:
            print('searching for lines')
            image = self.find_line(msg)
        
        self.cmd_pub.publish(self.twist)
        
        cv2.imshow("window", image)
        
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            self.cmd_pub.publish(Twist())
            rospy.signal_shutdown("Quit")

    def find_line(self, msg):
        # Convert to HSV
        image = self.bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # bounds for yellow
        lower_yellow = numpy.array([ 50,  50, 170])
        upper_yellow = numpy.array([120, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = image.shape
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            k_p = 0.01  # Proportional gain
            max_linear_x = 0.2  # Maximum linear velocity
            min_linear_x = 0.05
            self.twist.angular.z = -float(err) / 100
            self.twist.linear.x = max(min_linear_x, max_linear_x - k_p * abs(err))
            # End PID control
            
        return image

        
if __name__ == '__main__':
    rospy.init_node('follower_line_finder')
    follower = Follower()
    rospy.spin()