# coding:utf-8
from ddpg_brain import DDPG
from env import Env
import numpy as np


def run_task(env,net,max_episode):
    step = 0
    for episode in range(max_episode):
        env.reset()
        done = False
        while done is False:
            observation,reward,done = env.step()
            done = True
            step += 1
            print("Step: "+str(step+1))


if __name__ == '__main__':
    track_env = Env()
    track_ddpg = DDPG(1, 2, np.array([1.0]))
    run_task(track_env,track_ddpg,5)

