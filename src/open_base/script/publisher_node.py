#!/usr/bin/env python
import time
import math
import rospy
import roslib
import sys
import rospkg
import numpy as np
import os

from std_msgs.msg import String
from open_base.msg import Movement
from sensor_msgs.msg import Range
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import *

length_1 = 1
length_2 = 1
length_3 = 1
length_4 = 1
length_5 = 1
length_6 = 1
pos=0

rospy.init_node('Robot_control', anonymous=True)

def  callback_1(msg):
    global length_1
    length_1 = msg.range
    # rospy.loginfo("Sharp_1 = {} " .format(msg.range))

def sharp_1():
    rospy.Subscriber('/open_base/one_sonar', Range, callback_1)


def callback_2(msg):
    global length_2
    length_2 = msg.range
    # rospy.loginfo("Sharp_2 = {} " .format(msg.range))
def sharp_2():
    rospy.Subscriber('/open_base/two_sonar', Range, callback_2)



def callback_3(msg):
    global length_3
    length_3 = msg.range
    # rospy.loginfo("Sharp_3 = {} " .format(msg.range))
def sharp_3():
    rospy.Subscriber('/open_base/three_sonar', Range, callback_3)



def callback_4(msg):
    global length_4
    length_4 = msg.range
    # rospy.loginfo("Sharp_4 = {} " .format(msg.range))
def sharp_4():
    rospy.Subscriber('/open_base/for_sonar', Range, callback_4)




def callback_5(msg):
    global length_5
    length_5 = msg.range
    # rospy.loginfo("Sharp_5 = {} " .format(msg.range))
def sharp_5():
    rospy.Subscriber('/open_base/five_sonar', Range, callback_5)



def callback_6(msg):
    global length_6
    length_6 = msg.range
    # rospy.loginfo("Sharp_6 = {} " .format(msg.range))
def sharp_6():
    rospy.Subscriber('/open_base/six_sonar', Range, callback_6)



def move_platform(x,y,z):
    pub = rospy.Publisher('/open_base/command', Movement, queue_size=10)
    rate = rospy.Rate(10)
    vel = Movement()
    vel.wheel.v_left = x
    vel.wheel.v_back = y
    vel.wheel.v_right = z
    #rospy.loginfo('x: {} y:{} z:{}'.format(x,y,z))
    
    vel.movement = Movement.WHEEL
    pub.publish(vel)
    rate.sleep()


def callback(msg):
    global pos
    pos = msg.pose
   # rospy.loginfo("Pos = {} " .format(msg.pose))
def position():
    rospy.Subscriber("/gazebo/link_states", LinkStates,callback)



def move_R_R_R ():
    move_platform(pw1, pw2, pw3)






def set_pose():
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    objstate = SetModelStateRequest()  # Create an object of type SetModelStateRequest
    objstate.model_state.model_name = "open_base"
    objstate.model_state.pose.position.x = 0.0
    objstate.model_state.pose.position.y = 0.0
    objstate.model_state.pose.position.z = 0.0
    objstate.model_state.pose.orientation.w = 0
    objstate.model_state.pose.orientation.x = 0
    objstate.model_state.pose.orientation.y = 0
    objstate.model_state.pose.orientation.z = 0
    objstate.model_state.twist.linear.x = 0.0
    objstate.model_state.twist.linear.y = 0.0
    objstate.model_state.twist.linear.z = 0.0
    objstate.model_state.twist.angular.x = 0.0
    objstate.model_state.twist.angular.y = 0.0
    objstate.model_state.twist.angular.z = 0.0
    result = set_state_service(objstate)


set_pose()
file = open("myfile.txt", "w")
while True:
    move_platform(0.0, 0.3, -0.3)
    
    position()

    file.write(str(pos))

    sharp_1()
    sharp_2()
    sharp_3()
    sharp_4()
    sharp_5()
    sharp_6()
    rospy.loginfo('\n s1=: {} \n s2=: {} \n s3=: {} \n s4=: {} \n s5=: {} \n s6=: {}'.format(round(length_1,2),round(length_2,2),round(length_3,2),round(length_4,2),round(length_5,2),round(length_6,2)))
    if length_1 < 0.3 or length_2 < 0.3 or length_3 < 0.3 or length_4 < 0.3 or length_5 < 0.3 or length_6 < 0.5:
        move_platform(0.0, -0.3, 0.3)

        time.sleep(1)
        move_platform(0.3, 0.3, 0.3)
        time.sleep(1)
file.close()




    # sharp_1()
    # sharp_2()
    # sharp_3()
    # sharp_4()
    # sharp_5()
    # sharp_6()
    # move_platform(0.0, 0.5, -0.5)
    # time.sleep(5)
    # move_platform(0.5, 0.5, 0.5)
    # time.sleep(5)




