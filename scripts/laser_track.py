#! /usr/bin/env python

'''
Publishes a noisy x-axis position of a robot (map frame of reference) each time
a new laser scan msg arrives

- Node reads in laser scan topic, picks one laser ray pointing to the front of
  the robot.

- Calculates the robot's x-axis position in the map substracting the laser
  measurement from the position (provided as parameter) of one fixed obstacle in
  the front.

- Publishes the robot's x-axis position in the map coordinates as Float64 value.

- Assumes robot starts in map coord. 0, 0, 0.1 and moves towards negative
  x-axis.
  
- Assumes laser sensor is located in the center of the robot (no transform to
  robot base)

Author: Erick Suzart
Date: November 2020
'''

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import numpy as np

def laser_scan_callback(msg):
    global robot_x_axis_position, obstacle_front_x_axis, pub_

    # keep the minimum distance reading from 10 rays pointing to the front
    # second min is required to filter out 'inf' values, in that case 6 is used
    front_laser_ray = min(msg.ranges[355:364])
    rospy.logdebug("Distance to object in front (front_laser_ray): %s", front_laser_ray)

    # calculate robot position in the world considering the known position of an obstacle in front
    # example: position of obstacle: -5, laser_ray_reading = 3  => robot_x_axis_position = -2
    # This assumes/requires a robot moving straigt and parallel to x-axis, with orientation = [0,0,1,0]
    robot_x_axis_position = obstacle_front_x_axis + front_laser_ray
    rospy.logdebug("X position in map frame (robot_x_axis_position): %s", robot_x_axis_position)

    # generate noisy position from standard deviation for noise
    sd_trans = alpha_laser_scan*(front_laser_ray)
    robot_x_axis_position =  np.random.normal(robot_x_axis_position,sd_trans*sd_trans)
    pub_.publish(robot_x_axis_position)

if __name__ == '__main__':
    
    # global variables
    robot_x_axis_position = 0
    # laser scan noise parameter, specifies the noise in the laser scan readings
    alpha_laser_scan = 0.05
  
    rospy.init_node('position_from_laser_publisher', log_level=rospy.INFO)

    # get value of obstacle front in x-axis (world coordinates), this is used to
    # calculate the robot position in world coord
    if rospy.has_param("~obstacle_front_x_axis"):
        obstacle_front_x_axis = rospy.get_param("~obstacle_front_x_axis")
    else:
        obstacle_front_x_axis = -5.35
        rospy.logerr("No '/obstacle_front_x_axis' parameter found in parameter server, using default value '%s'", obstacle_front_x_axis)

    # get the topic where the laser scan msg is being published, gets a private
    # parameter (defined inside <node> tag)
    if rospy.has_param("~laser_scan_topic"):
        laser_scan_topic = rospy.get_param("~laser_scan_topic")
    else:
        # this is the topic name to which odom_msg_from_gazebo.py will publish by default
        laser_scan_topic = "/scan"
        rospy.logwarn("No '~laser_scan_topic' parameter found in parameter server, using default value %s", laser_scan_topic)

    sub_ = rospy.Subscriber(laser_scan_topic, LaserScan, laser_scan_callback)

    # publisher and topic name for measurement data
    pub_ = rospy.Publisher('/position_from_laser', Float64, queue_size = 1)

    # Print node start information
    print("-----------------------------------------------------\n")
    print("Started laser ray localization node")
    print("-----------------------------------------------------\n")
          
    rospy.spin()