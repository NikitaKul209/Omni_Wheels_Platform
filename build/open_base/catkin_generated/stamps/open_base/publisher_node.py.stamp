#!/usr/bin/env python
import rospy
import roslib
import sys

from std_msgs.msg import String
from open_base.msg import Movement
from sensor_msgs.msg import Range
from gazebo_msgs.msg import LinkStates


def callback(msg):
    rospy.loginfo("Message = ", msg.range)
def sharp_1():
    rospy.init_node('sharp_1', anonymous=True)
    rospy.Subscriber('/open_base/one_sonar', Range, callback)
    rospy.spin()



# def move_platform(x,y,z):
#     rospy.init_node('platform_move', anonymous=True)
#     pub = rospy.Publisher('/open_base/command', Movement, queue_size=10)
#     rate = rospy.Rate(10)
#     vel = Movement()
#     while True:
#         vel.wheel.v_left = x
#         vel.wheel.v_back = y
#         vel.wheel.v_right = z
#         rospy.loginfo('x: {} y:{} z:{}'.format(x,y,z))
#         vel.movement = Movement.WHEEL
#         pub.publish(vel)
#         rate.sleep()
# move_platform(0.0,-0.5,0.5)




# #def callback(data):
#     # rospy.loginfo(rospy.get_caller_id,data.data)
#
# def callback(msg):
#     rospy.loginfo("Message = ", msg.link_name)
# #def sharp_1():
#     rospy.init_node('Origin_link', anonymous=True)
#     rospy.Subscriber("/gazebo/link_states", LinkStates,callback)
#     rospy.spin()
#
# #  rospy.loginfo('дальность'.format(data))
# #