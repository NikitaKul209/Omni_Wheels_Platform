#!/usr/bin/env pythons
import time
import math
import rospy
import roslib
import sys
import rospkg

from functools import partial

from std_msgs.msg import String
from open_base.msg import Movement
from sensor_msgs.msg import Range
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import *
from stable_baselines3.common.env_checker import check_env


class Omni_Wheels_Platform():


        def __init__(self):
            # Показания дальномеров
            self.dists = [0.] * 6
            self.orient = 0
            self.position = 0
            self.sonar_topics = ['one_sonar','two_sonar','three_sonar','for_sonar','five_sonar','six_sonar']


            # rospy.logerr('Orient: {}'.format(self.orient))

            # create subscribers
            self.sub_funcs = [partial(self.callback_range,  sonar_index=i)
                              for i in range(len(self.sonar_topics))]
            self.subs_range = [rospy.Subscriber('/open_base/' + s, Range, f)
                               for s, f in zip(self.sonar_topics, self.sub_funcs)]
            self.sub_links = rospy.Subscriber("/gazebo/link_states", LinkStates,self.callback_links)

            self.pub_velocity = rospy.Publisher('/open_base/command', Movement, queue_size=10)

        def callback_range(self, msg, sonar_index):
            self.dists[sonar_index] = msg.range

        def move_platform(self,x,y,z):
            vel = Movement()
            vel.wheel.v_left = x
            vel.wheel.v_back = y
            vel.wheel.v_right = z
            #rospy.loginfo('x: {} y:{} z:{}'.format(x,y,z))
            vel.movement = Movement.WHEEL
            self.pub_velocity.publish(vel)

        def callback_links(self,msg):
            ind = msg.name.index('open_base::origin_link')
            pos = msg.pose[ind]
            x = pos.position.x
            y = pos.position.y
            z = pos.orientation.z
            w = pos.orientation.w
            self.position=[x,y]

            self.orient = int(math.atan2(z,w)*360/math.pi)
            if self.orient < 0:
                self.orient = (self.orient + 360)
                self.orient = int(self.orient)

            # rospy.loginfo("Pos = {} " .format(msg.pose))
            # rospy.loginfo("Orient = {:.3f} ".format(self.orient))
            # time.sleep(0.2)

        def reset_pose(self):
            self.Move_0_0_0()
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



        def Move_0_0_0(self):
            self.move_platform(0.0, 0.0, 0.0)
        def Move_0_0_L(self):
            self.move_platform(0.0, 0.0, -0.3)
        def Move_0_0_R(self):
            self.move_platform(0.0, 0.0, 0.3)
        def Move_0_L_0(self):
            self.move_platform(0.0, -0.3, 0.0)
        def Move_0_R_0(self):
            self.move_platform(0.0, 0.3, 0.0)
        def Move_0_L_L(self):
            self.move_platform(0.0, -0.3, -0.3)
        def Move_0_R_R(self):
            self.move_platform(0.0, 0.3, 0.3)
        def Move_0_L_R(self):
            self.move_platform(0.0, -0.3, 0.3)
        def Move_0_R_L(self):
            self.move_platform(0.0, 0.3, -0.3)
        def Move_L_0_0(self):
            self.move_platform(-0.3,0.0,0.0)
        def Move_R_0_0(self):
            self.move_platform(0.3,0.0,0.0)
        def Move_L_0_L(self):
            self.move_platform(-0.3,0.0,-0.3)
        def Move_R_0_R(self):
            self.move_platform(0.3, 0.0, 0.3)
        def Move_L_0_R(self):
            self.move_platform(-0.3,0.0,0.3)
        def Move_R_0_L(self):
            self.move_platform(0.3,0.0,-0.3)
        def Move_L_L_0(self):
            self.move_platform(-0.3,-0.3,0.0)
        def Move_R_R_0(self):
            self.move_platform(0.3,0.3,0.0)
        def Move_L_R_0(self):
            self.move_platform(-0.3,0.3,0.0)
        def Move_R_L_0(self):
            self.move_platform(0.3, -0.3, 0.0)
        def Move_L_L_L(self):
            self.move_platform(-0.3, -0.3, -0.3)
        def Move_R_R_R(self):
            self.move_platform(0.3, 0.3, 0.3)
        def Move_L_L_R(self):
            self.move_platform(-0.3, -0.3, 0.3)
        def Move_R_R_L(self):
            self.move_platform(0.3, 0.3, -0.3)
        def Move_L_R_R(self):
            self.move_platform(-0.3, 0.3, 0.3)
        def Move_R_L_L(self):
            self.move_platform(0.3, -0.3, -0.3)
        def Move_L_R_L(self):
            self.move_platform(-0.3, 0.3, -0.3)
        def Move_R_L_R(self):
            self.move_platform(0.3, -0.3, 0.3)
