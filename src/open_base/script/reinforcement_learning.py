#!/usr/bin/env python3

from main_robot_control import  Omni_Wheels_Platform
from datetime import datetime
import gym
import numpy as np
import rospy
import random
import  math
import time
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


        self.possible_actions = [('Move_0_0_0', self.robot.Move_0_0_0),
                                 ('Move_0_0_L', self.robot.Move_0_0_L),
                                 ('Move_0_0_R', self.robot.Move_0_0_R),
                                 ('Move_0_L_0', self.robot.Move_0_L_0),
                                 ('Move_0_R_0', self.robot.Move_0_R_0),
                                 ('Move_0_L_L', self.robot.Move_0_L_L),
                                 ('Move_0_R_R', self.robot.Move_0_R_R),
                                 ('Move_0_L_R', self.robot.Move_0_L_R),
                                 ('Move_0_R_L', self.robot.Move_0_R_L),
                                 ('Move_L_0_0', self.robot.Move_L_0_0),
                                 ('Move_R_0_0', self.robot.Move_R_0_0),
                                 ('Move_L_0_L', self.robot.Move_L_0_L),
                                 ('Move_R_0_R', self.robot.Move_R_0_R),
                                 ('Move_L_0_R', self.robot.Move_L_0_R),
                                 ('Move_R_0_L', self.robot.Move_R_0_L),
                                 ('Move_L_L_0', self.robot.Move_L_L_0),
                                 ('Move_R_R_0', self.robot.Move_R_R_0),
                                 ('Move_L_R_0', self.robot.Move_L_R_0),
                                 ('Move_R_L_0', self.robot.Move_R_L_0),
                                 ('Move_L_L_L', self.robot.Move_L_L_L),
                                 ('Move_R_R_R', self.robot.Move_R_R_R),
                                 ('Move_L_L_R', self.robot.Move_L_L_R),
                                 ('Move_R_R_L', self.robot.Move_R_R_L),
                                 ('Move_L_R_R', self.robot.Move_L_R_R),
                                 ('Move_R_L_L', self.robot.Move_R_L_L),
                                 ('Move_L_R_L', self.robot.Move_L_R_L),
                                 ('Move_R_L_R', self.robot.Move_R_L_R), ]

        self.start_position = [0, 0]
        self.current_position = [0, 0]
        self.action_distance = 0
        self.sum_distance = 0
        self.distance = 0
        self.speed = 0
        self.delta_y = 0
        self.delta_x = 0

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

        self.delta_y = 0
        self.delta_x = 0

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

        self.possible_actions[action][1]()

        time.sleep(0.1)
        obs = self._calculate_observation()
        reward = self.reward_calc()
        done = self.sum_distance >= 30

        self.current_position = self.robot.position
        self.action_distance = math.sqrt(((self.start_position[0] - self.current_position[0])**2) + ((self.start_position[1] - self.current_position[1])**2))
        self.distance = self.distance+self.action_distance
        reward = self.reward_calc()
        self.delta_x = self.current_position[0]-self.start_position[0]
        self.delta_y = self.current_position[1]-self.start_position[1]
        self.start_position = self.current_position

        return obs, reward, done, {}


    def reward_calc(self):

        if self.relative_orient == 1 and self.command == 0 and self.current_position[0] < self.start_position[0]:
            self.reward = 1+abs(self.delta_x)*100
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 0 and self.current_position[0] > self.start_position[0]:
            self.reward = -1


        elif self.relative_orient == 1 and self.command == 1 and self.current_position[0] > self.start_position[0]:
            self.reward = 1 + abs(self.delta_x) * 100
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 1 and self.current_position[0] < self.start_position[0]:
            self.reward = -1


        elif self.relative_orient == 1 and self.command == 2 and self.current_position[1] < self.start_position[1]:
            self.reward = 1 + abs(self.delta_y) * 100
            self.sum_distance = self.sum_distance + self.action_distance
        elif self.relative_orient == 1 and self.command == 2 and self.current_position[1] > self.start_position[1]:
            self.reward = -1


        elif self.relative_orient == 1 and self.command == 3 and self.current_position[1] > self.start_position[1]:
            self.reward = 1 + abs(self.delta_y) * 100
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
        self.speed = self.action_distance / 0.1
        # relative_velocity = 1 if self.speed >= 0.35 else (2 if 0.25 < self.speed < 0.35 else 3)


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

    # def render(self, mode='console',close=False):
    def my_render(self,i,epochs,sum_distance,old_obs,obs,possible_actions,start_robot_or,orient,possible_commands, reward):

        print("Итерация",i )
        print("Выбрано действий в итерации", epochs)
        # print("Проехал за действие", action_distance)
        print("Проехал в нужном направлении", sum_distance)
        # print("Скорость", speed)
        print("Начальное состояние", old_obs)
        print("Текущее состояние", obs)
        print("Выбранное действие", possible_actions)

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

    q_table = np.zeros([10 * len(env.possible_commands),27])
    old_q_table = np.zeros([10 * len(env.possible_commands),27])
    # q_table = np.loadtxt("q_table_test.txt")

    # Hyperparameters
    alpha = 0.5
    gamma = 0.1
    epsilon = 0.2

    # For plotting metrics
    all_epochs = []
    all_penalties = []

    for i in range(1, 100):
        obs = env.reset()

        epochs, penalties, reward, = 0, 0, 0
        random_flag = 0
        w=0
        done = False

        while not done:
            old_obs = obs
            print("##################################################################################")
            w=w+1
            old_q_table = q_table

            q_table_ind = (obs[0] - 1) * 10 + (obs[1] - 1)


            if random.uniform(0, 1) < epsilon:
                action = env.action_space.sample()  # Explore action space
                random_flag = 1
            else:
                action = np.argmax(q_table[q_table_ind]) # Exploit learned values
            if random_flag:
                print("Выбрано случайное действие", action)
                random_flag = 0
            else:
                print("Выбираю действие", action)
            print("Проехал всего", env.distance)

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

            env.my_render(i,epochs,env.sum_distance,old_obs,obs,env.possible_actions[action][0], env.start_robot_or,env.robot.orient,env.possible_commands[env.command][0], env.reward  )

            if w == 2000:
                np.savetxt("q_table_test.txt", q_table)
                w = 0

            rospy.sleep(0.001)
        if i % 100 == 0:
            print(f"Episode: {i}")

    print("Training finished.\n")
    print("Время обучения: ",datetime.now() - start_time)
    np.savetxt("q_table_test.txt", q_table)
    return q_table







def Q_Test():

    # Введите команду:
    # 0 - Север
    # 1 - Юг
    # 2 - Запад
    # 3 - Восток

    number_of_comand = 3

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
        # env.render()
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