#!/usr/bin/env python3

from main_robot_control import  Omni_Wheels_Platform
from datetime import datetime
import gym
import numpy as np
import rospy
import random
import  math
import time
import matplotlib.pyplot as mpl

global q_table
class GazeboEnv(gym.Env):
    """
    Custom Environment that follows gym interface.
    This is a simple env where the agent must learn to go always left.
    """
    # Because of google colab, we cannot implement the GUI ('human' render mode)
    metadata = {'render.modes': ['console']}
    # Define constants for clearer code

    def __init__(self, robot):

        super(GazeboEnv, self).__init__()

        self.robot = Omni_Wheels_Platform()

        self.NORTH = 90

        self.SOUTH = 270

        self.WEST = 180

        self.EAST = 0

        self.possible_commands = [('North', self.NORTH),
                                  ('South', self.SOUTH),
                                  ('West',  self.WEST),
                                  ('East',  self.EAST)]


        self.possible_actions = [('Move_0_0_0', self.robot.Move_0_0_0),# 0
                                 ('Move_0_0_L', self.robot.Move_0_0_L),# 1
                                 ('Move_0_0_R', self.robot.Move_0_0_R),# 2
                                 ('Move_0_L_0', self.robot.Move_0_L_0),# 3
                                 ('Move_0_R_0', self.robot.Move_0_R_0),# 4
                                 ('Move_0_L_L', self.robot.Move_0_L_L),# 5
                                 ('Move_0_R_R', self.robot.Move_0_R_R),# 6
                                 ('Move_0_L_R', self.robot.Move_0_L_R),# 7
                                 ('Move_0_R_L', self.robot.Move_0_R_L),# 8
                                 ('Move_L_0_0', self.robot.Move_L_0_0),# 9
                                 ('Move_R_0_0', self.robot.Move_R_0_0),# 10
                                 ('Move_L_0_L', self.robot.Move_L_0_L),# 11
                                 ('Move_R_0_R', self.robot.Move_R_0_R),# 12
                                 ('Move_L_0_R', self.robot.Move_L_0_R),# 13
                                 ('Move_R_0_L', self.robot.Move_R_0_L),# 14
                                 ('Move_L_L_0', self.robot.Move_L_L_0),# 15
                                 ('Move_R_R_0', self.robot.Move_R_R_0),# 16
                                 ('Move_L_R_0', self.robot.Move_L_R_0),# 17
                                 ('Move_R_L_0', self.robot.Move_R_L_0),# 18
                                 ('Move_L_L_L', self.robot.Move_L_L_L),# 19
                                 ('Move_R_R_R', self.robot.Move_R_R_R),# 20
                                 ('Move_L_L_R', self.robot.Move_L_L_R),# 21
                                 ('Move_R_R_L', self.robot.Move_R_R_L),# 22
                                 ('Move_L_R_R', self.robot.Move_L_R_R),# 23
                                 ('Move_R_L_L', self.robot.Move_R_L_L),# 24
                                 ('Move_L_R_L', self.robot.Move_L_R_L),# 25
                                 ('Move_R_L_R', self.robot.Move_R_L_R)] #26

        self.start_position = [0, 0]
        self.current_position = [0, 0]
        self.action_distance = 0
        self.sum_distance = 0
        self.distance = 0
        self.speed = 0
        self.delta_y = 0
        self.delta_x = 0
        self.reset_angle = 0
        self.pokazatel_kachestva = 0
        # self.ranges = self.robot.dists
        self.delta = 0
        n_actions = 27
        self.action_time = 0.15
        self.action_space = gym.spaces.Discrete(n_actions)
        self.observation_space = gym.spaces.Tuple((gym.spaces.Discrete(len(self.possible_commands)),
                                                   gym.spaces.Discrete(10)
                                                   ))


    def reset(self):

        self.robot.reset_pose(self.reset_angle)
        self.robot.orient = 0
        self.robot.position = [0,0]
        self.start_position = [0,0]
        self.current_position = [0, 0]
        self.distance = 0
        self.action_distance = 0
        self.sum_distance = 0

        self.delta_y = 0
        self.delta_x = 0

        # self.command = random.randrange(len(self.possible_commands))
        self.command = 0
        return self._calculate_observation()

    def step(self, action):
        self.start_robot_or = self.robot.orient

        self.possible_actions[action][1]()

        time.sleep(self.action_time)
        self.robot.Move_0_0_0()
        time.sleep(0.01)

        self.current_position = self.robot.position
        obs = self._calculate_observation()
        reward = self.reward_calc()
        done = self.sum_distance >= 10
        self.action_distance = math.sqrt(((self.start_position[0] - self.current_position[0])**2) + ((self.start_position[1] - self.current_position[1])**2))
        self.distance = self.distance+self.action_distance
        reward = self.reward_calc()


        self.delta_x = self.current_position[0]-self.start_position[0]

        self.delta_y = self.current_position[1]-self.start_position[1]
        self.start_position = self.current_position

        return obs, reward, done, {}


    def reward_calc(self):

        if self.relative_orient == 1 and self.command == 0 and self.current_position[0] < self.start_position[0]:
            self.reward = 2+abs(self.delta_x)*100
            self.pokazatel_kachestva = self.pokazatel_kachestva + self.reward
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 0 and self.current_position[0] > self.start_position[0]:
            self.reward = -1


        elif self.relative_orient == 1 and self.command == 1 and self.current_position[0] > self.start_position[0]:
            self.reward = 2 + abs(self.delta_x) * 100
            self.pokazatel_kachestva = self.pokazatel_kachestva + self.reward
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 1 and self.current_position[0] < self.start_position[0]:
            self.reward = -1


        elif self.relative_orient == 1 and self.command == 2 and self.current_position[1] < self.start_position[1]:
            self.reward = 2 + abs(self.delta_y) * 100
            self.pokazatel_kachestva = self.pokazatel_kachestva + self.reward
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 2 and self.current_position[1] > self.start_position[1]:
            self.reward = -1


        elif self.relative_orient == 1 and self.command == 3 and self.current_position[1] > self.start_position[1]:
            self.reward = 2 + abs(self.delta_y) * 100
            self.pokazatel_kachestva = self.pokazatel_kachestva + self.reward
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 3 and self.current_position[1] < self.start_position[1]:
            self.reward = -1

        else:
            self.delta = min(abs(np.max(self.possible_commands[self.command][1]) - self.robot.orient),
                             abs(np.min(self.possible_commands[self.command][1]) - self.robot.orient))
            self.reward = math.exp(-self.delta)

        return self.reward

    def _calculate_observation(self):

        _,goal_dir = self.possible_commands[self.command]

        self.relative_orient = self.find_state(10,self.robot.orient,goal_dir)
        self.speed = self.action_distance / self.action_time



        return [self.command, self.relative_orient]

    def find_state(self,number_of_states, current_or, goal_or):

        states = np.empty(number_of_states)
        for i in range(0, number_of_states):
            state = ((goal_or + (180 / number_of_states)) + ((360 / number_of_states) * i))
            if state > 359:
                state = state - 360
            states[i] = int(state)


        for i in range(0, number_of_states):
            if states[-1] < states[0]:
                if current_or in list(range(states[-1].astype(int), (states[0].astype(int)))):
                    relative_or = 1
                    break
            elif states[-1] > states[0]:
                if current_or in list(range(states[-1].astype(int), 360)) or current_or in list(
                        range(0, (states[0].astype(int)))):
                    relative_or = 1
                    break

            if i + 1 > 9:
                relative_or = number_of_states
                break
            elif (states[i].astype(int)) > (states[i + 1].astype(int)):
                if current_or in list(range(states[i].astype(int), 360)) or current_or in list(
                        range(0, (states[i + 1].astype(int)))):
                    relative_or = i + 2
                    break
            elif current_or in list(range(states[i].astype(int), (states[i + 1].astype(int)))):
                relative_or = i + 2
                break
        return relative_or

    def my_render(self,i,epochs,sum_distance,old_obs,obs,possible_actions,start_robot_or,orient,possible_commands, reward, random_flag, action):

        print("Итерация",i )
        print("Выбрано действий в итерации", epochs)
        # print("Проехал за действие", action_distance)
        print("Проехал в нужном направлении", sum_distance)
        print("Начальное состояние", old_obs)
        print("Текущее состояние", obs)
        if random_flag:
            print("Выбрано случайное действие",possible_actions, action)
        else:
            print("Выбираю действие",possible_actions, action)

        print('Начальная ориентация "{}"\nОриентация после выбора действия "{}"\nцель "{}"\nнаграда "{}"'
              .format(start_robot_or,
                      orient,
                      possible_commands,
                      reward))

    def close(self):
        pass


def Q_Learning():

    start_time = datetime.now()


    rospy.init_node('Robot_control', anonymous=True)
    robot = Omni_Wheels_Platform()
    env = GazeboEnv(robot)
    obs = env.reset()
    time.sleep(1)

    # q_table = np.zeros([10 * len(env.possible_commands),27])
    # old_q_table = np.zeros([10 * len(env.possible_commands),27])
    q_table = np.loadtxt("q_table_test.txt")

    # Hyperparameters
    alpha = 0.5
    gamma = 0.4
    epsilon = 0.2

    for i in range(1, 10):
        obs = env.reset()

        epochs, penalties, reward, = 0, 0, 0
        random_flag = 0
        w=0
        eps = 0


        done = False
        env.reset_angle = env.reset_angle + 45
        if env.reset_angle >= 360:
            env.reset_angle-=360


        graf, ax = mpl.subplots()
        ax.set_xlabel('Эпоха',fontsize = 20)
        ax.set_ylabel('Награда',fontsize = 20)

        while not done:
            mpl.scatter(epochs,reward)
            old_obs = obs
            print("##################################################################################")
            w=w+1
            eps=eps+1
            old_q_table = q_table
            q_table_ind = (obs[0] - 1) * 10 + (obs[1] - 1)

            if eps >= 100:
                epsilon = 0.0
            else:
                epsilon = 0.2
            if random.uniform(0, 1) < epsilon:
                action = env.action_space.sample()  # Explore action space
                random_flag = 1
            else:
                action = np.argmax(q_table[q_table_ind]) # Exploit learned values

            for j in range(1, 11):
                ind = (old_obs[0] - 1) * 10 + (j - 1)
                print('В состоянии "{}"  Наилучшее действие "{}"  Награда {}'.format(j, np.argmax(old_q_table[ind]),
                                                                                     (old_q_table[ind][action])))

            next_obs, reward, done, info = env.step(action)

            old_value = q_table[q_table_ind, action]

            q_table_new_ind = (obs[0] - 1) * 10 + (obs[1] - 1)

            next_max = np.max(q_table[q_table_new_ind])

            new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)

            q_table[q_table_ind, action] = new_value

            obs = next_obs

            epochs += 1

            env.my_render(i,epochs,env.sum_distance,old_obs,obs,env.possible_actions[action][0], env.start_robot_or,env.robot.orient,env.possible_commands[env.command][0], env.reward, random_flag, action  )

            random_flag = 0
            if w >= 200:
                np.savetxt("q_table_test.txt", q_table)
                w = 0

            rospy.sleep(0.001)
        if i % 100 == 0:
            print(f"Episode: {i}")

        # print("Показатель качества", env.pokazatel_kachestva/epochs)

    print("Training finished.\n")
    print("Время обучения: ",datetime.now() - start_time)
    np.savetxt("q_table_test.txt", q_table)
    return q_table


def Q_Test():

    q_table = np.loadtxt("q_table_test.txt")
    rospy.init_node('Robot_control', anonymous=True)
    robot = Omni_Wheels_Platform()
    env = GazeboEnv(robot)

    for i in range(1, 200):
        done = False
        obs = env.reset()
        time.sleep(1)

        env.reset_angle +=45
        if env.reset_angle >= 360:
            env.reset_angle -=360
        while not done:
            env.sum_distance += env.action_distance
            q_table_ind = (obs[0] - 1) * 10 + (obs[1] - 1)
            action = np.argmax(q_table[q_table_ind])
            print("Выбираю действие ",action)
            print("Проехал",env.sum_distance)

            next_obs, reward, done, info = env.step(action)
            done = env.sum_distance > 3
            obs = next_obs
            print(obs)
            rospy.sleep(0.01)




if __name__ == '__main__':
    try:
        Q_Learning()
        # Q_Test    ()

    except rospy.exceptions.ROSInterruptException:
        mpl.show()
        pass
exit(0)


