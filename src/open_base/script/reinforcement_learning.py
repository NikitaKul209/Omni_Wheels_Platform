#!/usr/bin/env python3

from main_robot_control import Omni_Wheels_Platform

import gym
import  numpy as np
import rospy
from gym import spaces

class GazeboEnv(gym.Env):
    """
    Custom Environment that follows gym interface.
    This is a simple env where the agent must learn to go always left.
    """
    # Because of google colab, we cannot implement the GUI ('human' render mode)
    metadata = {'render.modes': ['console']}
    # Define constants for clearer code

    Move_0_0_0 =0

    Move_0_0_L =1

    Move_0_0_R =2

    Move_0_L_0 =3

    Move_0_R_0 =4

    Move_0_L_L =5

    Move_0_R_R =6

    Move_0_L_R =7

    Move_0_R_L =8

    Move_L_0_0 =9

    Move_R_0_0 =10

    Move_L_0_L =11

    Move_R_0_R =12

    Move_L_0_R =13

    Move_R_0_L =14

    Move_L_L_0 =15

    Move_R_R_0 =16

    Move_L_R_0 =17

    Move_R_L_0 =18

    Move_L_L_L =19

    Move_R_R_R =20

    Move_L_L_R =21

    Move_R_R_L =22

    Move_L_R_R =23

    Move_R_L_L =24

    Move_L_R_L =25

    Move_R_L_R =26



    def __init__(self, robot):

        super(GazeboEnv, self).__init__()

        self.NORTH = list(range(80, 100))
        self.SOUTH = list(range(260, 280))
        self.WEST = list(range(170, 190))

        self.EAST = list(range(0, 10))
        self.EAST.extend(range(350,360))

        self.direction = self.NORTH
        self.robot = robot


        self.agent_pos = self.robot.orient
        n_actions = 27
        n_spaces = 4
        self.action_space = spaces.Discrete(n_actions)
        # The observation will be the coordinate of the agent
        # this can be described both by Discrete and Box space
        # self.observation_space = spaces.Box(1,self.agent_pos)



    def reset(self):
        """
        Important: the observation must be a numpy array
        :return: (np.array)
        """
        self.robot.reset_pose()
        # Initialize the agent at the right of the grid
        # here we convert to float32 to make it more general (in case we want to use continuous actions)
        return np.array([self.agent_pos]).astype(np.float32)

    def step(self, action):
        if action == self.Move_0_0_0:
            self.robot.Move_0_0_0()

        if action == self.Move_0_0_L:
            self.robot.Move_0_0_L()

        if action == self.Move_0_0_R:
            self.robot.Move_0_0_R()

        if action == self.Move_0_L_0:
            self.robot.Move_0_L_0()

        if action == self.Move_0_R_0:
            self.robot.Move_0_R_0()

        if action == self.Move_0_L_L:
            self.robot.Move_0_L_L()

        if action == self.Move_0_R_R:
            self.robot.Move_0_R_R()

        if action == self.Move_0_L_R:
            self.robot.Move_0_L_R()

        if action == self.Move_0_R_L:
            self.robot.Move_0_R_L()

        if action == self.Move_L_0_0:
            self.robot.Move_L_0_0()

        if action == self.Move_R_0_0:
            self.robot.Move_R_0_0()

        if action == self.Move_L_0_L:
            self.robot.Move_L_0_L()

        if action == self.Move_R_0_R:
            self.robot.Move_R_0_R()

        if action == self.Move_L_0_R:
            self.robot.Move_L_0_R()

        if action == self.Move_R_0_L:
            self.robot.Move_R_0_L()

        if action == self.Move_L_L_0:
            self.robot.Move_L_L_0()

        if action == self.Move_R_R_0:
            self.robot.Move_R_R_0()

        if action == self.Move_L_R_0:
            self.robot.Move_L_R_0()

        if action == self.Move_R_L_0:
            self.robot.Move_R_L_0()

        if action == self.Move_L_L_L:
            self.robot.Move_L_L_L()

        if action == self.Move_R_R_R:
            self.robot.Move_R_R_R()

        if action == self.Move_L_L_R:
            self.robot.Move_L_L_R()

        if action == self.Move_R_R_L:
            self.robot.Move_R_R_L()

        if action == self.Move_L_R_R:
            self.robot.Move_L_R_R()

        if action == self.Move_R_L_L:
            self.robot.Move_R_R_L()

        if action == self.Move_L_R_L:
            self.robot.Move_L_R_L()

        if action == self.Move_R_L_R:
            self.robot.Move_R_L_R()

        else:
            raise ValueError("Received invalid action={} which is not part of the action space".format(action))

        # Account for the boundaries of the grid
        self.agent_pos = np.clip(self.agent_pos)

        # Are we at the left of the grid?
        done = bool(self.agent_pos in self.NORTH)

        # Null reward everywhere except when reaching the goal (left of the grid)


        if self.agent_pos in self.NORTH:
            reward = 1

        if self.agent_pos in list(range(np.max(self.direction)+1, np.max(self.direction)+10,1)):
            reward = -1
            state = "few deviation right"
        if self.agent_pos in list(range(np.min(self.direction) - 1, np.min(self.direction) - 10,-1)):
            reward = -1
            state = "few deviation left"

        if self.agent_pos in list(range(np.max(self.direction) + 11, np.max(self.direction) + 20, 1)):
            reward = -2
            state = "few deviation right"
        if self.agent_pos in list(range(np.min(self.direction) - 11, np.min(self.direction) - 20, -1)):
            reward = -2
            state = "few deviation left"

            # if self.agent_pos - np.max(self.direction) > 20 or self.agent_pos - np.min(self.direction) < 20

#
#         # Optionally we can pass additional info, we are not using that for now
#         info = {}
#
#         return np.array([self.agent_pos]).astype(np.float32), reward, done, info
#
    def render(self, mode='console'):
        if mode != 'console':
            raise NotImplementedError()
        # agent is represented as a cross, rest as a dot
        print("." * self.agent_pos, end="")
        print("x", end="")


    def close(self):
        pass

def main():
    rospy.init_node('Robot_control', anonymous=True)
    robot = Omni_Wheels_Platform()
    env = GazeboEnv(robot)
    robot.reset_pose()


    while not rospy.is_shutdown():
        robot.Move_R_L_R()
        rospy.loginfo('\n'.join(['\n s{} : {:.3f}'.format(i+1, dist) for i,dist in enumerate(robot.dists)]))
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        pass
exit(0)












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


from stable_baselines3 import PPO, A2C # DQN coming soon
from stable_baselines3.common.env_util import make_vec_env

# Instantiate the env

# wrap it
# env = make_vec_env(lambda: env, n_envs=1)
# # model = ('MlpPolicy', env, verbose=1).learn(5000)
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
