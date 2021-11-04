#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math
import numpy as np
pi = math.pi
from geometry_msgs.msg import Twist

import tf

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2
def get_goals(my_color, rotation="CW"):
    if my_color == 'b':
        symbol = -1
        th     = pi
    else:
        symbol = 1
        th     = 0

    # rotation = 'CW'  # 回転方向を変える @en

    if rotation == 'CW':
        symbol_2 = -1
    else:
        symbol_2 = 1

    #print("get_goals", my_color, symbol, th, symbol_2)
         
    # 12x3 (x,y,yaw) 
    goal_xyyaw = np.array([ 
            [symbol * -0.8  , symbol * 0     , np.mod(0      + th ,2*pi) ], # (1） 
            [symbol * -0.8  , symbol * -0.2  ,-np.mod(-pi/4  + th ,2*pi) ], 
            [symbol * -0.8  , symbol * 0     , np.mod(pi/2   + th ,2*pi) ],
            [symbol * -0.8  , symbol *  0.2  , np.mod(pi/3   + th ,2*pi) ],
            [symbol * -0.5  , symbol *  0.2  , np.mod(0      + th ,2*pi) ],
            [symbol * 0     , symbol * 0.4   , np.mod(pi*4/5 + th ,2*pi) ], # (2)
            [symbol * 0     , symbol * 0.4   , np.mod(-pi/2  + th ,2*pi) ],
            [symbol * 0.2   , symbol * 0.4   , np.mod( 0     + th ,2*pi) ],
            [symbol * 0.2   , symbol * 0.4   ,-np.mod(pi/3   + th ,2*pi) ],
            [symbol * 0.5   , symbol *   0   , np.mod(pi     + th ,2*pi) ], #（3）
            [symbol * 0.5   , symbol *   0   , np.mod(-pi/2  + th ,2*pi) ],
            [symbol * 0.2   , symbol * -0.4  , np.mod( pi    + th ,2*pi) ],
            [symbol * 0.2   , symbol * -0.4  , np.mod(-pi/3  + th ,2*pi) ],
            [symbol * 0     , symbol * -0.4  , np.mod( 0     + th ,2*pi) ], # (4)
            [symbol * 0     , symbol * -0.4  , np.mod(-pi*4/5+ th ,2*pi) ],
            [symbol * 0     , symbol * -0.4  , np.mod( pi    + th ,2*pi) ],
            [symbol * -0.5  , symbol * 0     , np.mod( pi    + th ,2*pi) ], # (5) 
            [symbol * -1.40 , symbol * 0     , np.mod( 0     + th ,2*pi) ]
        ])     
    return goal_xyyaw 
 
 
class NaviBot(): 
    def __init__(self): 
        # velocity publisher 
        self.vel_pub  = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client   = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.my_color = rospy.get_param('~rside')

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  
        goal.target_pose.header.stamp    = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q = tf.transformations.quaternion_from_euler(0,0,yaw)
        # import pdb; pdb.set_trace()
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
        goal_xyyaw = get_goals(self.my_color)
        # import pdb; pdb.set_trace()
        
        for ii in range(goal_xyyaw.shape[0]):
            goal_tmp =goal_xyyaw[ii,:]
            print('self.setGoal(' +str(goal_tmp) + ')')
            self.setGoal(goal_tmp[0],goal_tmp[1],goal_tmp[2])

        print('END')        

if __name__ == '__main__':
    rospy.init_node('navirun')
    bot = NaviBot()
    bot.strategy()