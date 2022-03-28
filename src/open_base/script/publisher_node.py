#!/usr/bin/env python
import time

import rospy
import roslib
import sys
import rospkg

from std_msgs.msg import String
from open_base.msg import Movement
from sensor_msgs.msg import Range
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import *

global length_1
global length_2
global length_3
global length_4
global length_5
global length_6
length_1 = 0.4
rospy.init_node('Robot_control', anonymous=True)

def  callback_1(msg):
    length_1 = msg.range
    rospy.loginfo("Sharp_1 = {} " .format(msg.range))
    return length_1
def sharp_1():
    rospy.Subscriber('/open_base/one_sonar', Range, callback_1)



def callback_2(msg):
    length_2 = msg.range
    rospy.loginfo("Sharp_2 = {} " .format(msg.range))
def sharp_2():
    rospy.Subscriber('/open_base/two_sonar', Range, callback_2)



def callback_3(msg):
    length_3 = msg.range
    rospy.loginfo("Sharp_3 = {} " .format(msg.range))
def sharp_3():
    rospy.Subscriber('/open_base/three_sonar', Range, callback_3)



def callback_4(msg):
    length_4 = msg.range
    rospy.loginfo("Sharp_4 = {} " .format(msg.range))
def sharp_4():
    rospy.Subscriber('/open_base/for_sonar', Range, callback_4)




def callback_5(msg):
    length_5 = msg.range
    rospy.loginfo("Sharp_5 = {} " .format(msg.range))
def sharp_5():
    rospy.Subscriber('/open_base/five_sonar', Range, callback_5)



def callback_6(msg):
    length_6 = msg.range
    rospy.loginfo("Sharp_6 = {} " .format(msg.range))
def sharp_6():
    rospy.Subscriber('/open_base/six_sonar', Range, callback_6)



def move_platform(x,y,z):
    pub = rospy.Publisher('/open_base/command', Movement, queue_size=10)
    rate = rospy.Rate(10)
    vel = Movement()
    vel.wheel.v_left = x
    vel.wheel.v_back = y
    vel.wheel.v_right = z
    rospy.loginfo('x: {} y:{} z:{}'.format(x,y,z))
    vel.movement = Movement.WHEEL
    pub.publish(vel)
    rate.sleep()


def callback(msg):
    rospy.loginfo("Pos = {} " .format(msg.pose))
def Position():
    rospy.init_node('Origin_link', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates,callback)
    rospy.spin()

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
while True:

    move_platform(0.0, 0.5, -0.5)
    time.sleep(5)
    sharp_1()
    if length_1 < 0.6:
        move_platform(0.0, 0.0, 0.0)
        time.sleep(5)

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




