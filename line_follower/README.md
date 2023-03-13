# Line Follower
This is a Python program that runs on ROS (Robot Operating System) which utilizes the camera of a robot to follow a yellow line on the ground.

# Purpose
* Work with basic image processing and computer vision, PID control, and more complex behaviors
* Write a program to follow a distinct line on the ground using the robots camera
* Work with and process camera RGB data
* How to use line detection, image filtering, and contour detection to identify already known features

# How it works
* The "Follower" class uses OpenCV library to perform computer vision operations on the image.
* The image is converted from a ROS image message format to an OpenCV image format using cv_bridge.
* The yellow line is detected by * thresholding the image in the HSV color space using a lower and upper bound of yellow color.
* The search area is restricted to the bottom half of the image to avoid detecting other lines, such as the horizon or obstacles.
* A proportional-integral-derivative (PID) controller is used to adjust the robot's angular velocity to center the line in the image. The proportional gain (k_p) is set to 0.01. The maximum and minimum linear velocities are set to 0.2 and 0.05, respectively.
* If the line is lost, the robot rotates at the last angle. If there is no last angle or line detected, the robot searches for the entire view for a line and goes there.
* The class also defines a "find_line" method that performs the same operations as the "image_callback" method but without using the PID controller. This method is used when the line is lost and the robot needs to search for it.

# Requirements
To run this program, the following libraries are required:

* rospy
* cv2
* cv_bridge
* numpy
* geometry_msgs
* sensor_msgs

# Running the program
Clone the repository locally in your `catkin_ws/src`

`git clone https://github.com/naimulh247/cosi-119a/`

Run `catkin_make`

`cd ~/catkin_ws && catkin_make`

Make sure the file is executable with 

`chmod +x ~/catkin_ws/src/cosi-119a/line_follower/line_follower.py`

Run!!!

`rosrun cosi-119a line_follower.py`