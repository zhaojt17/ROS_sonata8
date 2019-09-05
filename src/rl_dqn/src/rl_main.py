# coding:utf-8
import numpy as np
import tensorflow as tf
import time
from RL_brain import DQNNetWork
from env import Env


def run_task(env=None, max_episode=100,net=None):
    start = time.clock()
    for episode in range(max_episode):

        # print time
        if (episode + 1) % 3 == 0:
            elapsed = (time.clock() - start)
            print("episode: " + str(episode + 1) + " Time Used: " + str(elapsed))
        # save weight
        if (episode + 1) % 5 == 0:
            net.save()


if __name__ == "__main__":
    env = Env()
    DQN = DQNNetWork(3, 2)
    run_task(env=env, net=DQN)
