#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
'''

import rospy
import random

from geometry_msgs.msg import Twist


class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # count variable
        self.value = 0

    def calcTwist(self):
        self.value += 1 
        if self.value < 8:
            x = 0.2
            th = 0
        elif self.value < 11:
            x = 0
            th = 1
        elif self.value < 13:
            x = 0.2
            th = 0
        elif self.value < 16:
            x = 0
            th = -1
        elif self.value < 19:
            x = 0.2
            th = 0
        elif self.value < 30:
            if self.value % 6 < 3:
                x = 0
                th = 0.2
            else:
                x = 0
                th = -0.2
        elif self.value == 30:
            x = 0
            th = 0
            self.value = 0
            

        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()

