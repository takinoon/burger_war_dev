#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

import math

pi = math.pi

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2

# --- target definition (r), refer http://localhost:5000/warstate ---
# the number means index of warstate json file.
# the target state is stored in all_field_score param (0:no one,1:mybot,2:enemy)
#
#      6                   8
#   [BLOCK]     14      [BLOCK]
#      7                   9
#          16 [BLOCK] 15
#      10                  12
#   [BLOCK]     17      [BLOCK]
#      11                  13
#
#  coordinate systemn
#            ^ X  blue bot
#            |
#            |
#     Y <----|-----
#            |
#            |
#            |    red bot
#
# ----------------------------------------
#        Back 0                  Back 3
#   R 2[enemy_bot(b)]L 1   R 5[my_bot(r)]L 4
#        Front                   Front
# ----------------------------------------


class NaviBot():
    def __init__(self):
        
        self.cnt_around = 0
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)




    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        

    def go_straight(self):
        twist = Twist()
        twist.linear.x = 0.2; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.vel_pub.publish(twist)
        r = rospy.Rate(5) # change speed 5fps
        while self.cnt_around < 6:
            self.vel_pub.publish(twist)
            self.cnt_around += 1
            print(self.cnt_around)
            r.sleep()
        self.cnt_around = 0
        twist.linear.x = 0
        twist.angular.z = 0.2
        while self.cnt_around < 6:
            self.vel_pub.publish(twist)
            self.cnt_around += 1
            print(self.cnt_around)
            r.sleep()
        self.cnt_around = 0
        twist.linear.x = 0.2
        twist.angular.z = 0
        while self.cnt_around < 6:
            self.vel_pub.publish(twist)
            self.cnt_around += 1
            print(self.cnt_around)
            r.sleep()
        self.cnt_around = 0
        twist.linear.x = 0
        twist.angular.z = 0
        self.vel_pub.publish(twist)






    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps

        while not rospy.is_shutdown():
            self.setGoal(-1,0,pi/4)
            self.setGoal(-1,0,7*pi/4)
            self.setGoal(-0.85,-0.5,5*pi/3)
            self.go_straight()
            self.setGoal(0,-1,3*pi/4)
            self.setGoal(0,-1,pi/4)
            self.setGoal(0.5,-0.85,pi/6)
            self.go_straight()
            self.setGoal(1,0,5*pi/4)
            self.setGoal(1,0,3*pi/4)
            self.setGoal(0.85,0.5,2*pi/3)
            self.go_straight()
            self.setGoal(0,1,7*pi/4)
            self.setGoal(0,1,5*pi/4)
            self.setGoal(-0.5,0.85,7*pi/6)
            self.go_straight()

            #内回り周回
            # self.setGoal(-1,0,pi/4)
            # self.setGoal(-1,0,7*pi/4)
            # self.setGoal(-0.3,-0.3,5*pi/3)
            # self.setGoal(0,-1,3*pi/4)
            # self.setGoal(0,-1,pi/4)
            # self.setGoal(0.3,-0.3,pi/6)
            # self.setGoal(1,0,5*pi/4)
            # self.setGoal(1,0,3*pi/4)
            # self.setGoal(0.3,0.3,2*pi/3)
            # self.setGoal(0,1,7*pi/4)
            # self.setGoal(0,1,5*pi/4)
            # self.setGoal(-0.3,0.3,7*pi/6)

        



if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()
