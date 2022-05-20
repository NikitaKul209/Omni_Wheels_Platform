#!/usr/bin/env python3

import math
import random

import numpy as np

import gym

# create custom environment
class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(CustomEnv, self).__init__()
    
        # constants:
        # list of directions the robot can move to (waiting + 4 directions with changes of coordinates)
        self.possible_actions = [('stop', 0, 0),
                                 ('move_up', 0, 1),
                                 ('move_down', 0, -1),
                                 ('move_left', -1, 0),
                                 ('move_right', 1,  0)]
        # robot input commands with goal coordinates
        self.possible_commands = [('cmd_stop', 0, 0),
                                  ('cmd_up', 0, 1),
                                  ('cmd_up_and_left', -1, 1),
                                  ('cmd_left', -1, 0),
                                  ('cmd_down_and_left', -1, -1),
                                  ('cmd_down', 0, -1),
                                  ('cmd_down_and_right', 1, -1),
                                  ('cmd_right', 1, 0),
                                  ('cmd_up_and_right', 1, 1)]
    
        # initialize inner variables of the environment
        self.reset()
    
        # required defines:
        # 1. action space: choose one of actions
        self.action_space = gym.spaces.Discrete(len(self.possible_actions))
        # 2. observation space: input command (Discrete) and two positions with respect to the goal
        # (by x and y axis: lower = 0, equal = 1, greater = 2)
        self.observation_space = gym.spaces.Tuple((gym.spaces.Discrete(len(self.possible_commands)),
                                                   gym.spaces.Discrete(3),
                                                   gym.spaces.Discrete(3)))

    def reset(self):
        # Reset the state of the environment to an initial state
        self.robot_x = 0
        self.robot_y = 0
        self.command = random.randrange(len(self.possible_commands))
        return self._calculate_observation()

    def step(self, action):
        # Execute one time step within the environment:
        # calculate new coordinates by the action
        _, dx, dy = self.possible_actions[action]
        self.robot_x += dx
        self.robot_y += dy
        # calculate reward
        reward = self._calculate_reward()
        # calculate next observation
        obs = self._calculate_observation()
        # check if processing finished
        done = (self.possible_commands[self.command][1] == self.robot_x 
                and self.possible_commands[self.command][2] == self.robot_y)
        # return observation and reward
        return obs, reward, done, {}
    
    def render(self, mode='human', close=False):
        # Render the environment to the screen
        print('Position is {} {}, goal is {} {}'
              .format(self.robot_x,
                      self.robot_y, 
                      self.possible_commands[self.command][1],
                      self.possible_commands[self.command][2]))


    def _calculate_reward(self):
        # distance between goal point and robot:
        _, goal_x, goal_y = self.possible_commands[self.command]
        dist = math.hypot(goal_x - self.robot_x, goal_y - self.robot_y)
        # calculate value of reward: reward(0) = 1, reward(big_number) -> 0
        return math.exp(-dist)
    
    def _calculate_observation(self):
        _, goal_x, goal_y = self.possible_commands[self.command]
        relative_pos_x = 1 if self.robot_x < goal_x else (2 if self.robot_x == goal_x else 3)
        relative_pos_y = 1 if self.robot_y < goal_y else (2 if self.robot_y == goal_y else 3)
        return [self.command, relative_pos_x, relative_pos_y]

# example for random movement
def random_movement():
    # create environment
    env = CustomEnv()
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


def q_learning_training():
    
    # create environment
    env = CustomEnv()
    # create q_table
    q_table = np.zeros([9 * len(env.possible_commands),
                        len(env.possible_actions)])
    # Hyperparameters
    alpha = 0.6
    gamma = 0.6
    epsilon = 0.1
    # for each epoch
    for i in range(1, 100):
        obs = env.reset()
        epochs, penalties, reward, = 0, 0, 0
        done = False
    
        while not done:
            # row in q_table to use
            q_table_ind = (obs[0]-1)*9 + (obs[1]-1)*3 + (obs[0]-1)
            
            if random.uniform(0, 1) < epsilon:
                action = env.action_space.sample() # Explore action space
            else:
                action = np.argmax(q_table[q_table_ind]) # Exploit learned values

            next_obs, reward, done, info = env.step(action) 
        
            old_value = q_table[q_table_ind, action]
            
            q_table_new_ind = (next_obs[0]-1)*9 + (next_obs[1]-1)*3 + (next_obs[0]-1)
            next_max = np.max(q_table[q_table_new_ind])
        
            new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
            q_table[q_table_ind, action] = new_value

            obs = next_obs
            epochs += 1
            env.render()
        if i % 100 == 0:
            print(f"Episode: {i}")

    print("Training finished.\n")
    return q_table


#random_movement()
q_table = q_learning_training()

# self.Move_0_0_0 = 0
#
# self.Move_0_0_L = 1
#
# self.Move_0_0_R = 2
#
# self.Move_0_L_0 = 3
#
# self.Move_0_R_0 = 4
#
# self.Move_0_L_L = 5
#
# self.Move_0_R_R = 6
#
# self.Move_0_L_R = 7
#
# self.Move_0_R_L = 8
#
# self.Move_L_0_0 = 9
#
# self.Move_R_0_0 = 10
#
# self.Move_L_0_L = 11
#
# self.Move_R_0_R = 12
#
# self.Move_L_0_R = 13
#
# self.Move_R_0_L = 14
#
# self.Move_L_L_0 = 15
#
# self.Move_R_R_0 = 16
#
# self.Move_L_R_0 = 17
#
# self.Move_R_L_0 = 18
#
# self.Move_L_L_L = 19
#
# self.Move_R_R_R = 20
#
# self.Move_L_L_R = 21
#
# self.Move_R_R_L = 22
#
# self.Move_L_R_R = 23
#
# self.Move_R_L_L = 24
#
# self.Move_L_R_L = 25
#
# self.Move_R_L_R = 26