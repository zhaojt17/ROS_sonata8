# coding:utf-8
import tensorflow as tf
import numpy as np


class DDPG(object):
    def __init__(self,
                 a_dim,
                 s_dim,
                 a_bound,
                 gamma=0.9,
                 lr_a=0.001,
                 lr_c=0.002,
                 tau=0.01,
                 memory_size=10000,
                 batch_size=32,
                 var_start=0.75,
                 var_replace_ratio=0.9995
                 ):
        self.a_dim, self.s_dim, self.a_bound = a_dim, s_dim, a_bound
        self.memory_size = memory_size
        self.gamma = gamma
        self.lr_a = lr_a
        self.lr_c = lr_c
        self.tau = tau
        self.memory = np.zeros((self.memory_size, s_dim * 2 + a_dim + 1), dtype=np.float32)
        self.memory_counter = 0
        self.batch_size = batch_size
        self.var = 2 * var_start * self.a_bound
        self.var_replace_ratio = var_replace_ratio
        self.sess = tf.Session()
        # put placeholder
        self.S = tf.placeholder(tf.float32, [None, s_dim], 's')
        self.S_ = tf.placeholder(tf.float32, [None, s_dim], 's_')
        self.R = tf.placeholder(tf.float32, [None, 1], 'r')
        # build network
        with tf.variable_scope('Actor'):
            self.a = self._build_a(self.S, scope='eval', trainable=True)
            self.a_ = self._build_a(self.S_, scope='target', trainable=False)
        with tf.variable_scope('Critic'):
            # assign self.a = a in memory when calculating q for td_error,
            # otherwise the self.a is from Actor when updating Actor
            self.q = self._build_c(self.S, self.a, scope='eval', trainable=True)
            self.q_ = self._build_c(self.S_, self.a_, scope='target', trainable=False)
        # networks parameters
        self.ae_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/eval')
        self.at_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Actor/target')
        self.ce_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/eval')
        self.ct_params = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope='Critic/target')
        # target net replacement
        self.soft_replace = [tf.assign(t, (1 - self.tau) * t + self.tau * e)
                             for t, e in zip(self.at_params + self.ct_params, self.ae_params + self.ce_params)]
        q_target = self.R + self.gamma * self.q_
        # in the feed_dic for the td_error, the self.a should change to actions in memory
        td_error = tf.losses.mean_squared_error(labels=q_target, predictions=self.q)
        self.ctrain = tf.train.AdamOptimizer(self.lr_c).minimize(td_error, var_list=self.ce_params)
        # calculate loss
        a_loss = - tf.reduce_mean(self.q)  # maximize the q
        self.atrain = tf.train.AdamOptimizer(self.lr_a).minimize(a_loss, var_list=self.ae_params)
        self.sess.run(tf.global_variables_initializer())

    def _build_a(self, s, scope, trainable):
        with tf.variable_scope(scope):
            net = tf.layers.dense(s, 30, activation=tf.nn.relu, name='l1', trainable=trainable)
            a = tf.layers.dense(net, self.a_dim, activation=tf.nn.tanh, name='a', trainable=trainable)
            return tf.multiply(a, self.a_bound, name='scaled_a')

    def _build_c(self, s, a, scope, trainable):
        with tf.variable_scope(scope):
            n_l1 = 30
            w1_s = tf.get_variable('w1_s', [self.s_dim, n_l1], trainable=trainable)
            w1_a = tf.get_variable('w1_a', [self.a_dim, n_l1], trainable=trainable)
            b1 = tf.get_variable('b1', [1, n_l1], trainable=trainable)
            net = tf.nn.relu(tf.matmul(s, w1_s) + tf.matmul(a, w1_a) + b1)
            return tf.layers.dense(net, 1, trainable=trainable)  # Q(s,a)

    def choose_action(self, s):
        action = self.sess.run(self.a, {self.S: s[np.newaxis, :]})[0]
        action = np.clip(np.random.normal(action, self.var), -self.a_bound, self.a_bound)
        return action

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, a, [r], s_))
        index = self.memory_counter % self.memory_size  # replace the old memory with new memory
        self.memory[index, :] = transition
        self.memory_counter += 1

    def learn(self):
        # soft target replacement
        self.sess.run(self.soft_replace)
        if self.memory_counter > self.memory_size:
            sample_index = np.random.choice(self.memory_size, size=self.batch_size)
            self.var *= self.var_replace_ratio
        else:
            sample_index = np.random.choice(self.memory_counter, size=self.batch_size)
        bt = self.memory[sample_index, :]
        bs = bt[:, :self.s_dim]
        ba = bt[:, self.s_dim: self.s_dim + self.a_dim]
        br = bt[:, -self.s_dim - 1: -self.s_dim]
        bs_ = bt[:, -self.s_dim:]
        self.sess.run(self.atrain, {self.S: bs})
        self.sess.run(self.ctrain, {self.S: bs, self.a: ba, self.R: br, self.S_: bs_})


