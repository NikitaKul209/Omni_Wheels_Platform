#!/usr/bin/env python3

from main_robot_control import  Omni_Wheels_Platform

import gym
import numpy as np
import rospy
from gym import spaces
import random
from IPython.display import clear_output
import  math
import time
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

        self.possible_commands = [('North', self.NORTH),
                                  ('South', self.SOUTH),
                                  ('West',  self.WEST),
                                  ('East',  self.EAST)]
        self.robot = Omni_Wheels_Platform()

        self.agent_pos = self.robot.orient
        self.ranges = self.robot.dists
        self.delta = 0
        n_actions = 27
        self.action_space = gym.spaces.Discrete(n_actions)
        self.observation_space = gym.spaces.Tuple((gym.spaces.Discrete(len(self.possible_commands)),
                                                   gym.spaces.Discrete(4)))


    def reset(self):
        """
        Important: the observation must be a numpy array
        :return: (np.array)
        """
        self.robot.reset_pose()
        self.robot.orient = 0

        self.command = random.randrange(len(self.possible_commands))
        return self._calculate_observation()

    def step(self, action):

        if action == self.Move_0_0_0:
            self.robot.Move_0_0_0()

        elif action == self.Move_0_0_L:
            self.robot.Move_0_0_L()

        elif action == self.Move_0_0_R:
            self.robot.Move_0_0_R()

        elif action == self.Move_0_L_0:
            self.robot.Move_0_L_0()

        elif action == self.Move_0_R_0:
            self.robot.Move_0_R_0()

        elif action == self.Move_0_L_L:
            self.robot.Move_0_L_L()

        elif action == self.Move_0_R_R:
            self.robot.Move_0_R_R()

        elif action == self.Move_0_L_R:
            self.robot.Move_0_L_R()

        elif action == self.Move_0_R_L:
            self.robot.Move_0_R_L()

        elif action == self.Move_L_0_0:
            self.robot.Move_L_0_0()

        elif action == self.Move_R_0_0:
            self.robot.Move_R_0_0()

        elif action == self.Move_L_0_L:
            self.robot.Move_L_0_L()

        elif action == self.Move_R_0_R:
            self.robot.Move_R_0_R()

        elif action == self.Move_L_0_R:
            self.robot.Move_L_0_R()

        elif action == self.Move_R_0_L:
            self.robot.Move_R_0_L()

        elif action == self.Move_L_L_0:
            self.robot.Move_L_L_0()

        elif action == self.Move_R_R_0:
            self.robot.Move_R_R_0()

        elif action == self.Move_L_R_0:
            self.robot.Move_L_R_0()

        elif action == self.Move_R_L_0:
            self.robot.Move_R_L_0()

        elif action == self.Move_L_L_L:
            self.robot.Move_L_L_L()

        elif action == self.Move_R_R_R:
            self.robot.Move_R_R_R()

        elif action == self.Move_L_L_R:
            self.robot.Move_L_L_R()

        elif action == self.Move_R_R_L:
            self.robot.Move_R_R_L()

        elif action == self.Move_L_R_R:
            self.robot.Move_L_R_R()

        elif action == self.Move_R_L_L:
            self.robot.Move_R_R_L()

        elif action == self.Move_L_R_L:
            self.robot.Move_L_R_L()

        elif action == self.Move_R_L_R:
            self.robot.Move_R_L_R()

        else:
            raise ValueError("Received invalid action={} which is not part of the action space".format(action))

        obs = self._calculate_observation()
        reward = self.reward_calc()
        done = self.robot.orient in self.possible_commands[self.command][1]
        time.sleep(0.1)

        # return observation and reward
        return obs, reward, done, {}


    def reward_calc(self):

        if self.robot.orient in self.possible_commands[self.command][1]:
            self.delta = 0
        else:
            self.delta = min(abs(np.max(self.possible_commands[self.command][1]) - self.robot.orient),
                             abs(np.min(self.possible_commands[self.command][1]) - self.robot.orient))
        self.reward = math.exp(-self.delta)
        return self.reward

    def _calculate_observation(self):

        _,goal_dir = self.possible_commands[self.command]
        relative_orient = 1 if self.robot.orient in goal_dir else (2 if self.delta in list(range(0, 40))
                                                else (3 if self.delta in list(range(41, 80)) else 4))
        return [self.command, relative_orient]


    def render(self, mode='console'):

        print('Orientation is {}, goal is {}, reward is {}'
              .format(self.robot.orient,
                      self.possible_commands[self.command][0],
                      self.reward))

    def close(self):
        pass





def Q_Learning():


    rospy.init_node('Robot_control', anonymous=True)
    robot = Omni_Wheels_Platform()
    env = GazeboEnv(robot)
    obs = env.reset()
    time.sleep(5)

    q_table = np.zeros([4 * len(env.possible_commands),27])

    # Hyperparameters
    alpha = 0.3
    gamma = 0.6
    epsilon = 0.1

    # For plotting metrics
    all_epochs = []
    all_penalties = []

    for i in range(1, 100):
        obs = env.reset()

        epochs, penalties, reward, = 0, 0, 0
        done = False

        while not done:
            # print(env.agent_pos)
            # print(obs)
            # print(robot.orient)
            # print(robot.dists)
            # rospy.loginfo('\n'.join(['\n s{} : {:.3f}'.format(i + 1, dist) for i, dist in enumerate(robot.dists)]))
            q_table_ind = (obs[0] - 1) * 4 + (obs[1] - 1)
            if random.uniform(0, 1) < epsilon:
                action = env.action_space.sample()  # Explore action space
            else:
                action = np.argmax(q_table[q_table_ind]) # Exploit learned values

            next_obs, reward, done, info = env.step(action)

            old_value = q_table[q_table_ind, action]
            q_table_new_ind = (obs[0] - 1) * 4 + (obs[1] - 1)
            next_max = np.max(q_table[q_table_new_ind])

            new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
            q_table[q_table_ind, action] = new_value

            obs = next_obs
            epochs += 1

            print(q_table)
            print(epochs)
            env.render()

        if i % 100 == 0:
            print(f"Episode: {i}")

    print("Training finished.\n")
    return q_table


def random_movement():
    # create environment
    rospy.init_node('Robot_control', anonymous=True)
    robot = Omni_Wheels_Platform
    env = GazeboEnv(robot)
    max_num_steps = 1000
    # reset environment
    env.reset()
    for step in range(max_num_steps):
        # get random action
        action = env.action_space.sample()
        # use this action
        obs, reward, done, info = env.step(action)
        # print info
        env.render()
        if done:
            # restart processing if the goal is achieved
            env.reset()
    env.close()




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
        Q_Learning()

    except rospy.exceptions.ROSInterruptException:
        pass
exit(0)






