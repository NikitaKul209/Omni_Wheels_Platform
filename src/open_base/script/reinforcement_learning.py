#!/usr/bin/env python3

from main_robot_control import  Omni_Wheels_Platform
from datetime import datetime
import gym
import numpy as np
import rospy
from gym import spaces
import random
from IPython.display import clear_output
import  math
import time
import pickle
global q_table
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

        self.NORTH = 90
        # self.NORTH = list(range(80, 100))
        self.SOUTH = 270
        # self.SOUTH = list(range(260, 280))
        self.WEST = 180
        # self.WEST = list(range(170, 190))
        self.EAST = 0
        # self.EAST = list(range(0, 10))
        # self.EAST.extend(range(350,360))

        self.possible_commands = [('North', self.NORTH),
                                  ('South', self.SOUTH),
                                  ('West',  self.WEST),
                                  ('East',  self.EAST)]
        self.robot = Omni_Wheels_Platform()

        self.start_position = [0, 0]
        self.current_position = [0, 0]
        self.action_distance = 0
        self.distance = 0
        self.speed = 0

        self.ranges = self.robot.dists
        self.delta = 0
        n_actions = 27
        self.action_space = gym.spaces.Discrete(n_actions)
        self.observation_space = gym.spaces.Tuple((gym.spaces.Discrete(len(self.possible_commands)),
                                                   gym.spaces.Discrete(10)
                                                   ))


    def reset(self):
        """
        Important: the observation must be a numpy array
        :return: (np.array)
        """
        self.robot.reset_pose()
        self.robot.orient = 0
        self.robot.position = [0,0]
        self.start_position = [0,0]
        self.current_position = [0, 0]
        self.distance = 0
        self.action_distance = 0
        self.sum_distance = 0
        self.sum_distance = 0

        self.command = random.randrange(len(self.possible_commands))
        return self._calculate_observation()

    def test_reset(self,number_of_comand):
        """
        Important: the observation must be a numpy array
        :return: (np.array)
        """
        self.robot.reset_pose()
        self.robot.orient = 0
        self.robot.position = [0, 0]
        self.start_position = [0, 0]
        self.current_position = [0, 0]
        self.distance = 0
        self.action_distance = 0
        self.sum_distance = 0
        self.sum_distance = 0


        self.command = number_of_comand
        return self._calculate_observation()

    def step(self, action):
        self.start_robot_or = self.robot.orient



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
        time.sleep(0.1)
        obs = self._calculate_observation()
        reward = self.reward_calc()
        # done = self.robot.orient in self.possible_commands[self.command][1]

        done = self.sum_distance >= 30

        self.current_position = self.robot.position
        self.action_distance = math.sqrt(((self.start_position[0] - self.current_position[0])**2) + ((self.start_position[1] - self.current_position[1])**2))
        self.distance = self.distance+self.action_distance
        reward = self.reward_calc()
        self.start_position = self.current_position


        # return observation and reward
        return obs, reward, done, {}


    def reward_calc(self):

        if self.relative_orient ==1 and self.command == 0 and self.current_position[0] < self.start_position[0]:
            self.reward = 0.5 + math.exp(self.action_distance*10000)
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 0 and self.current_position[0] > self.start_position[0]:
            self.reward = -1


        elif self.relative_orient == 1 and self.command == 1 and self.current_position[0] > self.start_position[0]:
            self.reward = 0.5 + math.exp(self.action_distance * 10000)
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 1 and self.current_position[0] < self.start_position[0]:
            self.reward = -1


        elif self.relative_orient == 1 and self.command == 2 and self.current_position[1] < self.start_position[1]:
            self.reward = 0.5 + math.exp(self.action_distance * 10000)
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 2 and self.current_position[1] > self.start_position[1]:
            self.reward = -1


        elif self.relative_orient == 1 and self.command == 3 and self.current_position[1] > self.start_position[1]:
            self.reward = 0.5 + math.exp(self.action_distance * 10000)
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 3 and self.current_position[1] < self.start_position[1]:
            self.reward = -1




        # if self.robot.orient in self.possible_commands[self.command][1] and self.speed >=0.35 and self.command == 0 and self.current_position[0] < self.start_position[0]:
        #     self.reward = 1000
        #     self.sum_distance = self.sum_distance + self.action_distance
        #
        # elif self.robot.orient in self.possible_commands[self.command][1] and self.speed >= 0.25 and self.command == 0 and self.current_position[0] < self.start_position[0]:
        #     self.reward = 100
        #     self.sum_distance = self.sum_distance + self.action_distance
        #
        #
        # elif self.robot.orient in self.possible_commands[self.command][1] and self.speed <= 0.25 and self.command == 0 and self.current_position[0] < self.start_position[0]:
        #     self.reward = 10
        #     self.sum_distance = self.sum_distance + self.action_distance



        else:
            self.delta = min(abs(np.max(self.possible_commands[self.command][1]) - self.robot.orient),
                             abs(np.min(self.possible_commands[self.command][1]) - self.robot.orient))
            #
            # if self.delta in list(range(161, 200)):
            #     self.reward = -3
            # elif self.delta in list(range(141, 160)):
            #     self.reward = -2
            # elif self.delta in list(range(121, 140)):
            #     self.reward = -1
            # else:
            self.reward = math.exp(-self.delta)

        return self.reward

    def _calculate_observation(self):

        _,goal_dir = self.possible_commands[self.command]

        self.relative_orient = self.find_state(10,self.robot.orient,goal_dir)


        # relative_orient = 1 if self.robot.orient in goal_dir else (2 if self.delta in list(range(1, 20))
        #                                         else (3 if self.delta in list(range(21, 40))
        #                                         else (4 if self.delta in list(range(41, 60))
        #                                         else (5 if self.delta in list(range(61, 80))
        #                                         else (6 if self.delta in list(range(81, 100))
        #                                         else (7 if self.delta in list(range(101, 120))
        #                                         else (8 if self.delta in list(range(121, 140))
        #                                         else (9 if self.delta in list(range(141, 160)) else 10 ))))))))
        self.speed = self.action_distance / 0.2
        relative_velocity = 1 if self.speed >= 0.35 else (2 if 0.25 < self.speed < 0.35 else 3)


        return [self.command, self.relative_orient, relative_velocity]

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

    def render(self, mode='console'):

        print('Начальная ориентация "{}"\nОриентация после выбора действия "{}"\nцель "{}"\nнаграда "{}"'
              .format(self.start_robot_or,
                      self.robot.orient,
                      self.possible_commands[self.command][0],
                      self.reward))

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
    q_table = np.loadtxt("q_table_test.txt")
    print(q_table)


    # Hyperparameters
    alpha = 0.7
    gamma = 0.6
    epsilon = 0.1

    # For plotting metrics
    all_epochs = []
    all_penalties = []

    for i in range(1, 100):
        obs = env.reset()


        epochs, penalties, reward, = 0, 0, 0
        w=0
        done = False

        while not done:
            old_obs = obs
            print("##################################################################################")
            # print("Старое состояние", obs)
            w=w+1
            # print(env.agent_pos)
            # print(obs)
            # print(robot.orient)
            # print(robot.dists)
            # rospy.loginfo('\n'.join(['\n s{} : {:.3f}'.format(i + 1, dist) for i, dist in enumerate(robot.dists)]))
            q_table_ind = (obs[0] - 1) * 10 + (obs[1] - 1)
            if random.uniform(0, 1) < epsilon:
                action = env.action_space.sample()  # Explore action space
            else:
                action = np.argmax(q_table[q_table_ind]) # Exploit learned values

            next_obs, reward, done, info = env.step(action)

            old_value = q_table[q_table_ind, action]
            q_table_new_ind = (obs[0] - 1) * 10 + (obs[1] - 1)
            next_max = np.max(q_table[q_table_new_ind])

            new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
            q_table[q_table_ind, action] = new_value

            obs = next_obs
            epochs += 1

            # print(q_table)



            print("Итерация",i)
            print("Выбрано действий в итерации" ,epochs)
            print("Проехал в нужном направлении", env.sum_distance)
            # print("Скорость", env.speed)
            print("Начальное состояние",old_obs)
            print("Текущее состояние",obs)
            print("Выбранное действие",action)
            print('Начальная ориентация "{}"\nОриентация после выбора действия "{}"\nцель "{}"\nнаграда "{}"'
                  .format(env.start_robot_or,
                          env.robot.orient,
                          env.possible_commands[env.command][0],
                          env.reward))

            print("##################################################################################")

            # for i in range(1,11):
            #     ind = (obs[0] - 1) * 10 + (i - 1)
            #     action = np.argmax(q_table[ind])
            #     print('"Я в состоянии "{}"  Нилучшее действие "{}"  Награда {}'.format(i,action, (q_table[ind][action])))
            #     print("Выбираю действие", action)

            # print("Проехал всего", env.distance)
            # env.render()

            if w == 1000:
                np.savetxt("q_table_test.txt", q_table)
                w = 0



            rospy.sleep(0.001)
        if i % 100 == 0:
            print(f"Episode: {i}")

    print("Training finished.\n")
    print("Время обучения: ",datetime.now() - start_time)

    # with open("q_table.txt","w") as file:
    #     file.write(str(q_table))

    np.savetxt("q_table_test.txt", q_table)
    return q_table







def Q_Test():

    # Введите команду:
    # 0 - Север
    # 1 - Юг
    # 2 - Запад
    # 3 - Восток

    number_of_comand = 0

    done = False
    q_table = np.loadtxt("q_table_test.txt")
    rospy.init_node('Robot_control', anonymous=True)
    robot = Omni_Wheels_Platform()
    env = GazeboEnv(robot)

    obs = env.test_reset(number_of_comand)
    while not done:

        q_table_ind = (obs[0] - 1) * 10 + (obs[1] - 1)
        # print((q_table[q_table_ind]))
        action = np.argmax(q_table[q_table_ind])
        print("Выбираю действие ",action)

        next_obs, reward, done, info = env.step(action)
        obs = next_obs
        # print(q_table)
        print(obs)
        env.render()
        rospy.sleep(0.01)
    # robot.Move_0_0_0()
















if __name__ == '__main__':
    try:
        Q_Learning()
        # Q_Test    ()

    except rospy.exceptions.ROSInterruptException:
        pass





exit(0)

































# def random_movement():
#     # create environment
#     rospy.init_node('Robot_control', anonymous=True)
#     robot = Omni_Wheels_Platform
#     env = GazeboEnv(robot)
#     max_num_steps = 1000
#     # reset environment
#     env.reset()
#     for step in range(max_num_steps):
#         # get random action
#         action = env.action_space.sample()
#         # use this action
#         obs, reward, done, info = env.step(action)
#         # print info
#         env.render()
#         if done:
#             # restart processing if the goal is achieved
#             env.reset()
#     env.close()




# def main():
#     rospy.init_node('Robot_control', anonymous=True)
#     robot = Omni_Wheels_Platform()
#     env = GazeboEnv(robot)
#     robot.reset_pose()
#
#
#     while not rospy.is_shutdown():
#         robot.Move_R_L_R()
#         rospy.loginfo('\n'.join(['\n s{} : {:.3f}'.format(i+1, dist) for i,dist in enumerate(robot.dists)]))
#         rospy.sleep(0.01)