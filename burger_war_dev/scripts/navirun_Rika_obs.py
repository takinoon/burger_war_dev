#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import rosparam
from math import radians, degrees, atan2

import tf

from geometry_msgs.msg import Twist

import tf
import tf

import numpy as np
import math

pi = math.pi
PI = math.pi

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

from Enemy_detector_obs import EnemyDetector

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


TARGET_TH = (
    (-PI/4, -PI/4, -PI/2, -PI/2, -PI*3/4, -PI*3/4, -PI*3/4, -PI*3/4),
    (-PI/4, -PI/4, -PI/3, -PI/2, -PI*3/5, -PI*3/4, -PI*3/4, -PI*3/4),
    (-PI/4, -PI/4, -PI/6,     0,   -PI/2, -PI*3/4,     -PI,  PI*3/4),
    (-PI/4, -PI/5,     0,     0,      PI,  PI*6/10,  PI*3/4,    PI/2),
    (    0,     0,  PI/2,  PI/2,      PI,  PI*3/4,  PI*3/4,    PI/2),
    (    0,  PI/4,  PI/3,  PI/2,  PI*5/6,  PI*3/4,  PI*3/4,  PI*3/4),
    ( PI/4,  PI/4,  PI/4,  PI/3,  PI*5/6,    PI/2,  PI*3/4,  PI*3/4),
    ( PI/4,  PI/4,  PI/4,  PI/4,    PI/3,    PI/2,  PI*3/4,  PI*3/4),
)
WIDTH = 1.2 * (2 **0.5) # [m]

class NaviBot():
    def __init__(self):

        self.robot_namespace = rospy.get_param('~robot_namespace')
        print(self.robot_namespace)
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #self.tfBuffer = tf.Buffer()

        self.enemy_detector = EnemyDetector()
        self.listener = tf.TransformListener()
        # robot state 'inner' or 'outer'
        self.state = 'inner' 
        # robot wheel rot 
        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.pose_x = 0
        self.pose_y = 0
        self.th = 0

        self.k = 0.5
        self.near_wall_range = 0.2  # [m]
        self.speed = 0.07


        self.pose_twist = Twist()
        self.pose_twist.linear.x = self.speed; self.pose_twist.linear.y = 0.; self.pose_twist.linear.z = 0.
        self.pose_twist.angular.x = 0.; self.pose_twist.angular.y = 0.; self.pose_twist.angular.z = 0.


        self.is_near_wall = False

        self.is_near_enemy = False
        self.enemy_direction = None
        self.enemy_dist = None
        self.near_enemy_twist = Twist()
        self.near_enemy_twist.linear.x = self.speed; self.near_enemy_twist.linear.y = 0.; self.near_enemy_twist.linear.z = 0.
        self.near_enemy_twist.angular.x = 0.; self.near_enemy_twist.angular.y = 0.; self.near_enemy_twist.angular.z = 0.

        self.is_initialized_pose = True

        # lidar scan
        self.scan = []

        # publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # subscriber

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


    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        map_name=self.robot_namespace+'/map'
        is_enemy_detected, x, y = self.getEnemyPos(map_name)
        print(is_enemy_detected)
        #self.setGoal(-0.5,0,0)
        #self.setGoal(-0.5,0,3.1415/2)
        
        #self.setGoal(0,0.5,0)
        #self.setGoal(0,0.5,3.1415)
        
        #self.setGoal(-0.5,0,-3.1415/2)
        
        #self.setGoal(0,-0.5,0)
        #self.setGoal(0,-0.5,3.1415)
        symbol = 1
        th = 0
        goal_xyyaw = np.array([ 
            [symbol * -1  , symbol * 0     , np.mod(pi/4 + th ,2*pi) ],
            [symbol * -1  , symbol * 0  , np.mod(pi*7/4 + th ,2*pi) ], 
            [symbol * -0.9  , symbol * -0.5     , np.mod(pi*5/3 + th ,2*pi) ],
            [symbol * 0  , symbol *  -1  , np.mod(pi*3/4 + th ,2*pi) ],
            [symbol * -0  , symbol *  -1  , np.mod(pi/4 + th ,2*pi) ],
            [symbol * 0.5     , symbol * -0.9   , np.mod(pi/6 + th ,2*pi) ], 
            [symbol * 1     , symbol * 0   , np.mod(pi*5/4  + th ,2*pi) ],
            [symbol * 1   , symbol * 0   , np.mod( pi*3/4     + th ,2*pi) ],
            [symbol * 0.9   , symbol * 0.5   ,np.mod(pi*2/3   + th ,2*pi) ],
            [symbol * 0   , symbol *   1   , np.mod(pi*7/4     + th ,2*pi) ], 
            [symbol * 0   , symbol *   1   , np.mod(pi*5/4  + th ,2*pi) ],
            [symbol * -0.5   , symbol * 0.9  , np.mod( pi*7/6    + th ,2*pi) ],
        ])     
        idx=0

        while(1):
            goal_val0 = goal_xyyaw[idx%len(goal_xyyaw)][0]
            goal_val1 = goal_xyyaw[idx%len(goal_xyyaw)][1]
            goal_val2 = goal_xyyaw[idx%len(goal_xyyaw)][2]
            # 敵の検出
            is_enemy_detected, enemy_dist, enemy_rad = self.getEnemyDistRad()
            # rospy.loginfo("is_enemy_detected:{} enemy_dist{} enemy_rad:{}".format(is_enemy_detected, enemy_dist, enemy_rad))

            #map_name=self.robot_namespace+'/map'
            if is_enemy_detected:
                print(enemy_rad)
                print(enemy_dist)
            else:
                print("no enemy")
            self.setGoal(goal_val0, goal_val1, goal_val2)
            idx += 1

    def getEnemyPos(self, frame):
        try:
            enemy_closest_name=self.robot_namespace+'/enemy_closest'
            trans_stamped = self.listener.lookupTransform(frame, enemy_closest_name, rospy.Time())
            #trans = trans_stamped.transform
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            return False, 0, 0
        return True, trans_stamped[0], trans_stamped[1]

    def getEnemyDistRad(self):
        try:
            # <class 'geometry_msgs.msg._TransformStamped.TransformStamped'>
            base_footprint_name=self.robot_namespace+'/base_footprint'
            enemy_closest_name=self.robot_namespace+'/enemy_closest'
            trans_stamped = self.listener.lookupTransform(base_footprint_name, enemy_closest_name, rospy.Time())
            #trans = trans_stamped.transform
            # trans = self.tfBuffer.lookup_transform('enemy_closest', "base_footprint", rospy.Time(), rospy.Duration(4))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # rate.sleep()
            # rospy.logwarn(e)
            return False, 0, 0
        # rospy.loginfo(trans)
        #print(trans_stamped)
        
        dist = (trans_stamped[0][0]**2 + trans_stamped[0][1]**2)**0.5
        rad = atan2(trans_stamped[0][1], trans_stamped[0][0])
        # print ("trans.translation.x:{}, trans.translation.y:{}".format(trans.translation.x, trans.translation.y))

        # rot = trans.rotation
        # rad = tf.transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]

        return True, dist, rad
    
if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()