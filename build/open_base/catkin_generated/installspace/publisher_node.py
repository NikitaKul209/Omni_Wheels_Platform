#!/usr/bin/env python3
import time
import math
import gym
import rospy
import roslib
import sys
import rospkg
import numpy as np

from functools import partial


from std_msgs.msg import String
from open_base.msg import Movement
from sensor_msgs.msg import Range
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import *
import numpy as np
import gym
from gym import spaces
from stable_baselines3.common.env_checker import check_env



class Omni_Wheels_Platform():
        # os.startfile('/home/nikita/omni_ws/src/open_base/script/launch.py')
        # subprocess.Popen(['python3","launch.py", shell=True])
        # time.sleep(5)

        def __init__(self):
            # Показания дальномеров
            self.dists = [0.] * 6
            self.sonar_topics = ['one_sonar','two_sonar','three_sonar','for_sonar','five_sonar','six_sonar']

            self.orient = 0


            rospy.init_node('Robot_control', anonymous=True)
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
            z = pos.orientation.z
            w = pos.orientation.w
            self.orient = math.atan2(z, w)*180/math.pi
            #rospy.loginfo("Pos = {} " .format(msg.pose))
            #rospy.loginfo("Orient = {:.3f} ".format(self.orient))

        def reset_pose(self):
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


        # def O_0_0(self):
        #     self.move_platform(0.0, 0.0, 0.0)
        #
        # def O_0_L(self, pwm3):
        #     self.move_platform(0.0, 0.0, -pwm3)
        #
        # def O_0_R(self, pwm3):
        #     self.move_platform(0.0, 0.0, pwm3)
        #
        # def O_L_0(self, pwm2):
        #     self.move_platform(0.0, -pwm2, 0.0)
        #
        # def O_R_0(self, pwm2):
        #     self.move_platform(0.0, pwm2, 0.0)
        #
        # def O_L_L(self, pwm2, pwm3):
        #     self.move_platform(0.0, -pwm2, -pwm3)
        #
        # def O_R_R(self, pwm2, pwm3):
        #     self.move_platform(0.0, pwm2, pwm3)
        #
        # def O_L_R(self, pwm2, pwm3):
        #     self.move_platform(0.0, -pwm2, pwm3)
        #
        # def O_R_L(self, pwm2, pwm3):
        #     self.move_platform(0.0, pwm2, -pwm3)
        #
        # def L_0_0(self, pwm1):
        #     self.move_platform(-pwm1,0.0,0.0)
        #
        # def R_0_0(self, pwm1):
        #     self.move_platform(pwm1,0.0,0.0)
        #
        # def L_0_L(self, pwm1,pwm3):
        #     self.move_platform(-pwm1,0.0,-pwm3)
        #
        # def R_0_R(self, pwm1,pwm3):
        #     self.move_platform(pwm1, 0.0, pwm3)
        #
        # def L_0_R(self, pwm1,pwm3):
        #     self.move_platform(-pwm1,0.0,pwm3)
        #
        # def R_0_L(self, pwm1,pwm3):
        #     self.move_platform(pwm1,0.0,-pwm3)
        #
        # def L_L_0(self, pwm1,pwm2):
        #     self.move_platform(-pwm1,-pwm2,0.0)
        #
        # def R_R_0(self, pwm1,pwm2):
        #     self.move_platform(pwm1,pwm2,0.0)
        #
        # def L_R_0(self, pwm1,pwm2):
        #     self.move_platform(-pwm1,pwm2,0.0)
        #
        # def R_L_0(self, pwm1, pwm2):
        #     self.move_platform(pwm1, -pwm2, 0.0)
        #
        # def L_L_L(self, pwm1, pwm2,pwm3):
        #     self.move_platform(-pwm1, -pwm2, -pwm3)
        #
        # def R_R_R(self, pwm1, pwm2, pwm3):
        #     self.move_platform(pwm1, pwm2, pwm3)
        #
        # def L_L_R(self, pwm1, pwm2, pwm3):
        #     self.move_platform(-pwm1, -pwm2, pwm3)
        #
        # def R_R_L(self, pwm1, pwm2, pwm3):
        #     self.move_platform(pwm1, pwm2, -pwm3)
        #
        # def L_R_R(self, pwm1, pwm2, pwm3):
        #     self.move_platform(-pwm1, pwm2, pwm3)
        #
        # def R_L_L(self, pwm1, pwm2, pwm3):
        #     self.move_platform(pwm1, -pwm2, -pwm3)
        #
        # def L_R_L(self, pwm1, pwm2, pwm3):
        #     self.move_platform(-pwm1, pwm2, -pwm3)
        #
        # def R_L_R(self, pwm1, pwm2, pwm3):
        #     self.move_platform(pwm1, -pwm2, pwm3)

        def Move_0_0_0(self):
            self.move_platform(0.0, 0.0, 0.0)

        def Move_0_0_L(self):
            self.move_platform(0.0, 0.0, -0.5)

        def Move_0_0_R(self):
            self.move_platform(0.0, 0.0, 0.5)

        def Move_0_L_0(self):
            self.move_platform(0.0, -0.5, 0.0)

        def Move_0_R_0(self):
            self.move_platform(0.0, 0.5, 0.0)

        def Move_0_L_L(self):
            self.move_platform(0.0, -0.5, -0.5)

        def Move_0_R_R(self):
            self.move_platform(0.0, 0.5, 0.5)

        def Move_0_L_R(self):
            self.move_platform(0.0, -0.5, 0.5)

        def Move_0_R_L(self):
            self.move_platform(0.0, 0.5, -0.5)

        def Move_L_0_0(self):
            self.move_platform(-0.5,0.0,0.0)

        def Move_R_0_0(self):
            self.move_platform(0.5,0.0,0.0)

        def Move_L_0_L(self):
            self.move_platform(-0.5,0.0,-0.5)

        def Move_R_0_R(self):
            self.move_platform(0.5, 0.0, 0.5)

        def Move_L_0_R(self):
            self.move_platform(-0.5,0.0,0.5)

        def Move_R_0_L(self):
            self.move_platform(0.5,0.0,-0.5)

        def Move_L_L_0(self):
            self.move_platform(-0.5,-0.5,0.0)

        def Move_R_R_0(self):
            self.move_platform(0.5,0.5,0.0)

        def Move_L_R_0(self):
            self.move_platform(-0.5,0.5,0.0)

        def Move_R_L_0(self):
            self.move_platform(0.5, -0.5, 0.0)

        def Move_L_L_L(self):
            self.move_platform(-0.5, -0.5, -0.5)

        def Move_R_R_R(self):
            self.move_platform(0.5, 0.5, 0.5)

        def Move_L_L_R(self):
            self.move_platform(-0.5, -0.5, 0.5)

        def Move_R_R_L(self):
            self.move_platform(0.5, 0.5, -0.5)

        def Move_L_R_R(self):
            self.move_platform(-0.5, 0.5, 0.5)

        def Move_R_L_L(self):
            self.move_platform(0.5, -0.5, -0.5)

        def Move_L_R_L(self):
            self.move_platform(-0.5, 0.5, -0.5)

        def Move_R_L_R(self):
            self.move_platform(0.5, -0.5, 0.5)


def main():
    robot = Omni_Wheels_Platform()
    robot.reset_pose()
    robot.move_platform(0.6, 0.6, 0.6)
    while not rospy.is_shutdown():
        #rospy.loginfo('\n'.join(['s{} : {:.3f}'.format(i+1, dist) for i,dist in enumerate(robot.dists)]))
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass
exit(0)

#
#
# set_pose()
# file = open("myfile.txt", "w")
# while True:
#     move_platform(0.0, 0.3, -0.3)
#
#     position()
#
#     file.write(str(pos))
#
#     sharp_1()
#     sharp_2()
#     sharp_3()
#     sharp_4()
#     sharp_5()
#     sharp_6()
#     rospy.loginfo('\n s1=: {} \n s2=: {} \n s3=: {} \n s4=: {} \n s5=: {} \n s6=: {}'.format(round(length_1,2),round(length_2,2),round(length_3,2),round(length_4,2),round(length_5,2),round(length_6,2)))
#     if length_1 < 0.3 or length_2 < 0.3 or length_3 < 0.3 or length_4 < 0.3 or length_5 < 0.3 or length_6 < 0.5:
#         move_platform(0.0, -0.3, 0.3)
#
#
#         time.sleep(1)
#         move_platform(0.3, 0.3, 0.3)
#         time.sleep(1)
# file.close()
#



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


# class GazeboEnv(gym.Env):
#     """
#     Custom Environment that follows gym interface.
#     This is a simple env where the agent must learn to go always left.
#     """
#     # Because of google colab, we cannot implement the GUI ('human' render mode)
#     metadata = {'render.modes': ['console']}
#     # Define constants for clearer code
#
#
#
#     NORTH = range (80,100)
#
#     SOUTH = range (80,100)
#
#     WEST = range (80,100)
#
#     EAST = range (10,100)
#
#
#     Move_0_0_0 =0
#
#     Move_0_0_L =1
#
#     Move_0_0_R =2
#
#     Move_0_L_0 =3
#
#     Move_0_R_0 =4
#
#     Move_0_L_L =5
#
#     Move_0_R_R =6
#
#     Move_0_L_R =7
#
#     Move_0_R_L =8
#
#     Move_L_0_0 =9
#
#     Move_R_0_0 =10
#
#     Move_L_0_L =11
#
#     Move_R_0_R =12
#
#     Move_L_0_R =13
#
#     Move_R_0_L =14
#
#     Move_L_L_0 =15
#
#     Move_R_R_0 =16
#
#     Move_L_R_0 =17
#
#     Move_R_L_0 =18
#
#     Move_L_L_L =19
#
#     Move_R_R_R =20
#
#     Move_L_L_R =21
#
#     Move_R_R_L =22
#
#     Move_L_R_R =23
#
#     Move_R_L_L =24
#
#     Move_L_R_L =25
#
#     Move_R_L_R =26
#
#     def __init__(self, grid_size=10):
#         super(GazeboEnv, self).__init__()
#
#         # Size of the 1D-grid
#         self.grid_size = grid_size
#         # Initialize the agent at the right of the grid
#         self.agent_pos = grid_size - 1
#
#         # Define action and observation space
#         # They must be gym.spaces objects
#         # Example when using discrete actions, we have two: left and right
#         n_actions = 27
#         self.action_space = spaces.Discrete(n_actions)
#         # The observation will be the coordinate of the agent
#         # this can be described both by Discrete and Box space
#         self.observation_space = spaces.Box(low=0, high=self.grid_size,
#                                             shape=(1,), dtype=np.float32)
#
# robot = Omni_Wheels_Platform
#
#     def reset(self):
#         """
#         Important: the observation must be a numpy array
#         :return: (np.array)
#         """
#         robot.set_pose()
#         # Initialize the agent at the right of the grid
#         self.agent_pos = self.grid_size - 1
#         # here we convert to float32 to make it more general (in case we want to use continuous actions)
#         return np.array([self.agent_pos]).astype(np.float32)
#
#     def step(self, action):
#         if action == self.Move_0_0_0:
#             robot.Move_0_0_0()
#
#         if action == self.Move_0_0_L:
#             robot.Move_0_0_L()
#
#         if action == self.Move_0_0_R:
#             robot.Move_0_0_R()
#
#         if action == self.Move_0_L_0:
#             robot.Move_0_L_0()
#
#         if action == self.Move_0_R_0:
#             robot.Move_0_R_0()
#
#         if action == self.Move_0_L_L:
#             robot.Move_0_L_L()
#
#         if action == self.Move_0_R_R:
#             robot.Move_0_R_R()
#
#         if action == self.Move_0_L_R:
#             robot.Move_0_L_R()
#
#         if action == self.Move_0_R_L:
#             robot.Move_0_R_L()
#
#         if action == self.Move_L_0_0:
#             robot.Move_L_0_0()
#
#         if action == self.Move_R_0_0:
#             robot.Move_R_0_0()
#
#         if action == self.Move_L_0_L:
#             robot.Move_L_0_L()
#
#         if action == self.Move_R_0_R:
#             robot.Move_R_0_R()
#
#         if action == self.Move_L_0_R:
#             robot.Move_L_0_R()
#
#         if action == self.Move_R_0_L:
#             robot.Move_R_0_L()
#
#         if action == self.Move_L_L_0:
#             robot.Move_L_L_0()
#
#         if action == self.Move_R_R_0:
#             robot.Move_R_R_0()
#
#         if action == self.Move_L_R_0:
#             robot.Move_L_R_0()
#
#         if action == self.Move_R_L_0:
#             robot.Move_R_L_0()
#
#         if action == self.Move_L_L_L:
#             robot.Move_L_L_L()
#
#         if action == self.Move_R_R_R:
#             robot.Move_R_R_R()
#
#         if action == self.Move_L_L_R:
#             robot.Move_L_L_R()
#
#         if action == self.Move_R_R_L:
#             robot.Move_R_R_L()
#
#         if action == self.Move_L_R_R:
#             robot.Move_L_R_R()
#
#         if action == self.Move_R_L_L:
#             robot.Move_R_R_L()
#
#         if action == self.Move_L_R_L:
#             robot.Move_L_R_L()
#
#         if action == self.Move_R_L_R:
#             robot.Move_R_L_R()
#
#         else:
#             raise ValueError("Received invalid action={} which is not part of the action space".format(action))
#
#         # Account for the boundaries of the grid
#         self.agent_pos = np.clip(self.agent_pos, 0, self.grid_size)
#
#         # Are we at the left of the grid?
#         done = bool(self.agent_pos == 0)
#
#         # Null reward everywhere except when reaching the goal (left of the grid)
#         reward = 1 if self.agent_pos == 0 else 0
#
#         # Optionally we can pass additional info, we are not using that for now
#         info = {}
#
#         return np.array([self.agent_pos]).astype(np.float32), reward, done, info
#
#     def render(self, mode='console'):
#         if mode != 'console':
#             raise NotImplementedError()
#         # agent is represented as a cross, rest as a dot
#         print("." * self.agent_pos, end="")
#         print("x", end="")
#         print("." * (self.grid_size - self.agent_pos))
#
#     def close(self):
#         pass
#
#
# class GoLeftEnv(gym.Env):
#     """
#     Custom Environment that follows gym interface.
#     This is a simple env where the agent must learn to go always left.
#     """
#     # Because of google colab, we cannot implement the GUI ('human' render mode)
#     metadata = {'render.modes': ['console']}
#     # Define constants for clearer code
#     LEFT = 0
#     RIGHT = 1
#
#     def __init__(self, grid_size=10):
#         super(GoLeftEnv, self).__init__()
#
#         # Size of the 1D-grid
#         self.grid_size = grid_size
#         # Initialize the agent at the right of the grid
#         self.agent_pos = grid_size - 1
#
#         # Define action and observation space
#         # They must be gym.spaces objects
#         # Example when using discrete actions, we have two: left and right
#         n_actions = 2
#         self.action_space = spaces.Discrete(n_actions)
#         # The observation will be the coordinate of the agent
#         # this can be described both by Discrete and Box space
#         self.observation_space = spaces.Box(low=0, high=self.grid_size,
#                                             shape=(1,), dtype=np.float32)
#
#     def reset(self):
#         """
#         Important: the observation must be a numpy array
#         :return: (np.array)
#         """
#         robot.set_pose()
#         # Initialize the agent at the right of the grid
#         self.agent_pos = self.grid_size - 1
#         # here we convert to float32 to make it more general (in case we want to use continuous actions)
#         return np.array([self.agent_pos]).astype(np.float32)
#
#     def step(self, action):
#         if action == self.LEFT:
#             self.agent_pos -= 1
#         elif action == self.RIGHT:
#             self.agent_pos += 1
#         else:
#             raise ValueError("Received invalid action={} which is not part of the action space".format(action))
#
#         # Account for the boundaries of the grid
#         self.agent_pos = np.clip(self.agent_pos, 0, self.grid_size)
#
#         # Are we at the left of the grid?
#         done = bool(self.agent_pos == 0)
#
#         # Null reward everywhere except when reaching the goal (left of the grid)
#         reward = 1 if self.agent_pos == 0 else 0
#
#         # Optionally we can pass additional info, we are not using that for now
#         info = {}
#
#         return np.array([self.agent_pos]).astype(np.float32), reward, done, info
#
#     def render(self, mode='console'):
#         if mode != 'console':
#             raise NotImplementedError()
#         # agent is represented as a cross, rest as a dot
#         print("." * self.agent_pos, end="")
#         print("x", end="")
#         print("." * (self.grid_size - self.agent_pos))
#
#     def close(self):
#         pass

#
# env = GoLeftEnv()
# # If the environment don't follow the interface, an error will be thrown
# check_env(env, warn=True)
#
#
# env = GoLeftEnv(grid_size=10)
#
# obs = env.reset()
# env.render()
#
# print(env.observation_space)
# print(env.action_space)
# print(env.action_space.sample())
#
# GO_LEFT = 0
# # Hardcoded best agent: always go left!
# n_steps = 20
# for step in range(n_steps):
#   print("Step {}".format(step + 1))
#   obs, reward, done, info = env.step(GO_LEFT)
#   print('obs=', obs, 'reward=', reward, 'done=', done)
#   env.render()
#   if done:
#     print("Goal reached!", "reward=", reward)
#     break
#
#
# from stable_baselines3 import PPO, A2C # DQN coming soon
# from stable_baselines3.common.env_util import make_vec_env
#
# # Instantiate the env
# env = GoLeftEnv(grid_size=10)
# # wrap it
# env = make_vec_env(lambda: env, n_envs=1)
# model = A2C('MlpPolicy', env, verbose=1).learn(5000)
#
# obs = env.reset()
# n_steps = 20
# for step in range(n_steps):
#   action, _ = model.predict(obs, deterministic=True)
#   print("Step {}".format(step + 1))
#   print("Action: ", action)
#   obs, reward, done, info = env.step(action)
#   print('obs=', obs, 'reward=', reward, 'done=', done)
#   env.render(mode='console')
#   if done:
#     # Note that the VecEnv resets automatically
#     # when a done signal is encountered
#     print("Goal reached!", "reward=", reward)
#     break