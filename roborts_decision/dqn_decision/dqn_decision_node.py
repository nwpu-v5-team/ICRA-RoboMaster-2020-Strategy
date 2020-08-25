#!/usr/bin/env python3
import os
import shutil
import time
from timeit import default_timer

import rospy

from dqn_network import DeepQNetwork
from firefly_env import FireflyEnv

ACTION_NUM = [3, 6, 3, 6]

my_color = rospy.get_param('my_color')


def monitor_time(name: str, func, *parameter):
    start = default_timer()
    value = func(*parameter)
    end = default_timer()
    print(name, "use time: %f" % (end - start))
    return value


def copy_data(path: str):
    if os.path.exists(path):
        all_list = os.listdir(path)
        if len(all_list) > 0:
            tar_p = os.path.join(path, '..', 'copy',
                                 time.strftime("%m-%d-%H:%M:%S", time.localtime())
                                 + my_color + os.path.split(path)[1] + '-copy')
            shutil.copytree(path, tar_p)
            shutil.rmtree(path)
            os.mkdir(path)
    else:
        print('copy path is not exist')
    pass


def train(env, load_model_data):
    """ Start match and train """
    step = 0
    # rate = rospy.Rate(10)

    global ACTION_NUM
    RL = DeepQNetwork(ACTION_NUM, len(env.get_obs()),
                      learning_rate=0.01,
                      reward_decay=0.9,
                      e_greedy=0.9,
                      replace_target_iter=200,
                      memory_size=3000,
                      e_greedy_increment=0.0001,
                      output_tensorboard=True,
                      load_model_data=load_model_data)

    trainingTime = 300
    for episode in range(trainingTime):
        observation = monitor_time("reset", env.reset)

        while not rospy.is_shutdown():
            action = monitor_time("choose action", RL.choose_action, observation)
            print("dqn本次策略：", action)

            observation_new, reward, done, info = monitor_time(
                "step", env.step,
                action[0] + ACTION_NUM[0] * action[1] + ACTION_NUM[0] * ACTION_NUM[1] * action[2]
                + ACTION_NUM[0] * ACTION_NUM[1] * ACTION_NUM[2] * action[3])

            monitor_time("RL store transition", RL.store_transition, observation, action, reward, observation_new)

            startTime = 50
            learn_frequency = 5
            if (step > startTime) and (step % learn_frequency == 0):
                monitor_time("learn", RL.learn)

            observation = observation_new

            if done:
                break

            if step % 1200 == 1:
                monitor_time("store network", RL.store_network,
                             'MyModel' + my_color + time.strftime("%m-%d-%H:%M:%S", time.localtime()))

            monitor_time("save reward and loss data", RL.save_reward_loss)
            step += 1
            # rate.sleep()
        monitor_time("store network", RL.store_network,
                     'MyModel-' + my_color + time.strftime("%m-%d-%H:%M:%S", time.localtime()))


def match(env):
    """ Start a match without training """
    step = 0
    # rate = rospy.Rate(10)

    global ACTION_NUM
    RL = DeepQNetwork(ACTION_NUM, len(env.get_obs()),
                      learning_rate=0.01,
                      reward_decay=0.9,
                      e_greedy=0.93,
                      replace_target_iter=200,
                      memory_size=3000,
                      e_greedy_increment=None,
                      output_tensorboard=False,
                      load_model_data=True)

    observation = env.start_game()

    while not rospy.is_shutdown():
        action = RL.choose_action(observation)
        observation, reward, done, info = env.step(action)

        step += 1
        # rate.sleep()


def main():
    """ The entrypoint of running and dqn training """
    rospy.init_node('dqn_decision_node')
    env = FireflyEnv()

    copy_data(os.path.join(os.path.dirname(__file__) + '/data/txt'))

    is_match = rospy.get_param('is_match')
    load_model_data = rospy.get_param('load_model_data')

    try:
        if is_match:
            match(env)
        else:
            train(env, load_model_data)

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
