"""
Deep Deterministic Policy Gradient (DDPG), Reinforcement Learning.
DDPG is Actor Critic based algorithm.
Pendulum example.
View more on my tutorial page: https://morvanzhou.github.io/tutorials/
Using:
tensorflow 1.0
gym 0.8.0
"""

from ddpg_brain import DDPG
import numpy as np
import gym
import time


#####################  hyper parameters  ####################

MAX_EPISODES = 200
MAX_EP_STEPS = 200

RENDER = False
ENV_NAME = 'Pendulum-v0'


###############################  training  ####################################

env = gym.make(ENV_NAME)
env = env.unwrapped
env.seed(1)

s_dim = env.observation_space.shape[0]
a_dim = env.action_space.shape[0]
a_bound = env.action_space.high
print(a_bound)
ddpg = DDPG(a_dim, s_dim, a_bound)

var = 3  # control exploration
t1 = time.time()
for i in range(MAX_EPISODES):
    s = env.reset()
    ep_reward = 0
    for j in range(MAX_EP_STEPS):
        if RENDER:
            env.render()
        # Add exploration noise
        a = ddpg.choose_action(s)
        s_, r, done, info = env.step(a)
        ddpg.store_transition(s, a, r / 10, s_)
        if ddpg.memory_counter > ddpg.memory_size:
            ddpg.learn()
        s = s_
        ep_reward += r
        if j == MAX_EP_STEPS-1:
            print('Episode:', i, ' Reward: %i' % int(ep_reward), 'Explore: %.2f' % ddpg.var, )
            if ep_reward > -300:
                RENDER = True
            break
print('Running time: ', time.time() - t1)
