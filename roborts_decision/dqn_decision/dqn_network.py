import os

import numpy as np
import tensorflow.compat.v1 as tf
import matplotlib.pyplot as plt

np.random.seed(1)
tf.set_random_seed(1)


class DeepQNetwork:
    """
    dqn 深度强化学习

    Using:
        Tensorflow: 1.15

    create:
        策略数，环境维度

    choose_action:
        get observation, return action（0 - (num_actions-1))

    store_transition: store data as memory
        get observation, action, reward, observation_new
        return none

    learn: start one learn with memory
        get none, return none

    plot_cost: print picture at the end
        get none, return none

    get_reward_history
        get none, return reward list

    get_cost_history
        get none, return cost list

    store_network: save parameters at save path
        get save path, return none

    load_network: load parameters at load path
        get load path, return none
    """

    def __init__(
            self,
            nums_actions: list,
            num_features: int,
            learning_rate=0.01,
            reward_decay=0.9,
            e_greedy=0.9,
            replace_target_iter=300,
            memory_size=500,
            batch_size=32,
            e_greedy_increment=None,
            output_tensorboard=False,
            load_model_data=False
    ):
        """
        初始化
        :param nums_actions: 列表，列表中每个数字表示各组别动作个数
        :param num_features: 环境维度
        :param learning_rate: 学习率
        :param reward_decay: 奖励影响率（gamma）
        :param e_greedy: 选择最优策略的概率（当直接使用训练好的模型时，可设置为1）
        :param replace_target_iter: 隔多少拍更新一次tar网络参数
        :param memory_size: 记忆池的容量
        :param batch_size: 每次训练的组的数据大小
        :param e_greedy_increment: 是否不断减小e_greedy
        :param output_tensorboard: 是否输出tensorboard图
        :param load_model_data: 是否直接从指定路径加载网络参数
        """
        self._reward = 0.0

        self._n_actions = nums_actions  # 输出多少个action的q_value
        self._n_features = num_features  # 特征个数（环境）

        self._lr = learning_rate  # 学习率
        self._gamma = reward_decay  # 奖励的递减数
        self._epsilon_max = e_greedy  # 选择最优策略的概率（相对探索）
        self._replace_target_iter = replace_target_iter  # 多少步更新一次tar
        self._memory_size = memory_size  # 记忆池的容量
        self._batch_size = batch_size
        self._epsilon_increment = e_greedy_increment  # 不断缩小随机的范围
        self._epsilon = 0 if e_greedy_increment is not None else self._epsilon_max
        self._output_tensorboard = output_tensorboard

        # total learning step   记录学习次数，来更新tar参数
        self._learn_step_counter = 0

        # initialize zero memory [s, a, r, s_]  初始化记忆
        self._memory = np.zeros((self._memory_size, self._n_features * 2 + len(self._n_actions) + 1))

        self._sess = tf.Session()
        self._build_net()

        if load_model_data:
            self.load_network()
        else:
            # consist of [target_net, evaluate_net] 创建tar和eva网络
            self._sess.run(tf.global_variables_initializer())

        # 定义替换tar参数运算
        t_params = tf.get_collection('target_net_params')  # 提取tar的参数
        e_params = tf.get_collection('eval_net_params')  # 提取eva的参数
        self._replace_target_op = [tf.assign(t, e) for t, e in zip(t_params, e_params)]  # 更新tar

        # 输出tensorboard 文件
        if self._output_tensorboard:
            self._summary_writer = tf.summary.FileWriter(os.path.dirname(__file__) + "/logs/", self._sess.graph)
            self._merged = tf.summary.merge_all()

        # 记录cost和reward的变化，用于plot出来可视化
        self._cost11_his = []
        self._cost12_his = []
        self._cost21_his = []
        self._cost22_his = []
        self._reward_hit = []

        # memary
        self._memory_counter = 0
        pass

    def _build_net(self):
        """ Build the neuron network """
        # ------------------ build evaluate_net ------------------
        self._s = tf.placeholder(tf.float32, [None, self._n_features], name='s')  # input
        self._q_target11 = tf.placeholder(tf.float32, [None, self._n_actions[0]], name='Q_target_11')
        self._q_target12 = tf.placeholder(tf.float32, [None, self._n_actions[1]], name='Q_target_12')
        self._q_target21 = tf.placeholder(tf.float32, [None, self._n_actions[2]], name='Q_target_21')
        self._q_target22 = tf.placeholder(tf.float32, [None, self._n_actions[3]], name='Q_target_22')

        with tf.variable_scope('eval_net'):
            # c_names(collections_names) are the collections to store variables
            c_names, w_initializer, b_initializer = ['eval_net_params', tf.GraphKeys.GLOBAL_VARIABLES], \
                                                    tf.random_normal_initializer(0., 0.3), tf.constant_initializer(
                0.1)  # config of layers
            n_l1, n_l2, n_l3, n_l4, n_l5, n_l6, n_l7, n_l8, n_l9 = 180, 360, 720, 910, 720, 288, 72, 36, 18

            l11_8, l12_8, l21_8, l22_8 = self.build_sub_network(w_initializer, b_initializer, c_names, n_l1, n_l2, n_l3,
                                                                n_l4, n_l5, n_l6, n_l7, n_l8, n_l9)

            self._q_eval11, self._q_eval12, self._q_eval21, self._q_eval22 = \
                self.build_output_net(w_initializer, b_initializer, c_names, l11_8, l12_8, l21_8, l22_8, n_l9)

        with tf.variable_scope('loss'):
            self._loss11 = tf.reduce_mean(tf.squared_difference(self._q_target11, self._q_eval11))
            self._loss12 = tf.reduce_mean(tf.squared_difference(self._q_target12, self._q_eval12))
            self._loss21 = tf.reduce_mean(tf.squared_difference(self._q_target21, self._q_eval21))
            self._loss22 = tf.reduce_mean(tf.squared_difference(self._q_target22, self._q_eval22))
            if self._output_tensorboard:
                tf.summary.scalar('loss11', self._loss11)
                tf.summary.scalar('loss12', self._loss12)
                tf.summary.scalar('loss21', self._loss21)
                tf.summary.scalar('loss22', self._loss22)
        with tf.variable_scope('train'):
            self._train_op11 = tf.train.AdamOptimizer(self._lr).minimize(self._loss11)
            self._train_op12 = tf.train.AdamOptimizer(self._lr).minimize(self._loss12)
            self._train_op21 = tf.train.AdamOptimizer(self._lr).minimize(self._loss21)
            self._train_op22 = tf.train.AdamOptimizer(self._lr).minimize(self._loss22)

        # ------------------ build target_net ------------------
        self._s_ = tf.placeholder(tf.float32, [None, self._n_features], name='s_')  # input
        with tf.variable_scope('target_net'):
            # c_names(collections_names) are the collections to store variables
            c_names = ['target_net_params', tf.GraphKeys.GLOBAL_VARIABLES]

            l11_8, l12_8, l21_8, l22_8 = self.build_sub_network(w_initializer, b_initializer, c_names, n_l1, n_l2, n_l3,
                                                                n_l4, n_l5, n_l6, n_l7, n_l8, n_l9)

            self._q_next11, self._q_next12, self._q_next21, self._q_next22 = \
                self.build_output_net(w_initializer, b_initializer, c_names, l11_8, l12_8, l21_8, l22_8, n_l9)

    def build_output_net(self, w_initializer, b_initializer, c_names, l11_8, l12_8, l21_8, l22_8, n_l9):
        with tf.variable_scope('l11_10'):
            w10 = tf.get_variable('w10', [n_l9, self._n_actions[0]], initializer=w_initializer, collections=c_names)
            b10 = tf.get_variable('b10', [1, self._n_actions[0]], initializer=b_initializer, collections=c_names)
            q_11 = tf.matmul(l11_8, w10) + b10
            pass
        with tf.variable_scope('l12_10'):
            w10 = tf.get_variable('w10', [n_l9, self._n_actions[1]], initializer=w_initializer, collections=c_names)
            b10 = tf.get_variable('b10', [1, self._n_actions[1]], initializer=b_initializer, collections=c_names)
            q_12 = tf.matmul(l12_8, w10) + b10
            pass
        with tf.variable_scope('l21_10'):
            w10 = tf.get_variable('w10', [n_l9, self._n_actions[2]], initializer=w_initializer, collections=c_names)
            b10 = tf.get_variable('b10', [1, self._n_actions[2]], initializer=b_initializer, collections=c_names)
            q_21 = tf.matmul(l21_8, w10) + b10
            pass
        with tf.variable_scope('l22_10'):
            w10 = tf.get_variable('w10', [n_l9, self._n_actions[3]], initializer=w_initializer, collections=c_names)
            b10 = tf.get_variable('b10', [1, self._n_actions[3]], initializer=b_initializer, collections=c_names)
            q_22 = tf.matmul(l22_8, w10) + b10
            pass
        return q_11, q_12, q_21, q_22

    def build_sub_network(self, w_initializer, b_initializer, c_names, n_l1, n_l2, n_l3, n_l4, n_l5, n_l6, n_l7, n_l8,
                          n_l9):
        l5 = DeepQNetwork._create_dense(self._s, [n_l1, n_l2, n_l3, n_l4, n_l5], 1, 'l',
                                        {'initializer': w_initializer, 'collections': c_names},
                                        {'initializer': b_initializer, 'collections': c_names})
        l1_7 = DeepQNetwork._create_dense(l5, [n_l6, n_l7], 6, 'l1_',
                                          {'initializer': w_initializer, 'collections': c_names},
                                          {'initializer': b_initializer, 'collections': c_names})
        l2_7 = DeepQNetwork._create_dense(l5, [n_l6, n_l7], 6, 'l2_',
                                          {'initializer': w_initializer, 'collections': c_names},
                                          {'initializer': b_initializer, 'collections': c_names})
        l11_8 = DeepQNetwork._create_dense(l1_7, [n_l8, n_l9], 8, 'l11_',
                                           {'initializer': w_initializer, 'collections': c_names},
                                           {'initializer': b_initializer, 'collections': c_names})
        l12_8 = DeepQNetwork._create_dense(l1_7, [n_l8, n_l9], 8, 'l12_',
                                           {'initializer': w_initializer, 'collections': c_names},
                                           {'initializer': b_initializer, 'collections': c_names})
        l21_8 = DeepQNetwork._create_dense(l2_7, [n_l8, n_l9], 8, 'l21_',
                                           {'initializer': w_initializer, 'collections': c_names},
                                           {'initializer': b_initializer, 'collections': c_names})
        l22_8 = DeepQNetwork._create_dense(l2_7, [n_l8, n_l9], 8, 'l22_',
                                           {'initializer': w_initializer, 'collections': c_names},
                                           {'initializer': b_initializer, 'collections': c_names})
        return l11_8, l12_8, l21_8, l22_8

    @staticmethod
    def _create_dense(input_tensor: tf.Tensor, net_size: list, start_layer_id: int,
                      layer_name: str, w_dir: dict, b_dir: dict):
        """
        创建指定网络结构的CNN网络(默认各层都使用relu）
        输出层需要另外建立
        :param input_tensor: 网络输入, 要求为一个tensor:shape=[1, n]
        :param net_size: 网络规格, 不包括输入层
        :return: tensor
        """
        this_input = input_tensor
        input_node_num = input_tensor.get_shape().as_list().pop()
        layer_id = start_layer_id

        for this_node_num in net_size:
            with tf.variable_scope(layer_name + str(layer_id)):
                w = tf.get_variable('w' + str(layer_id), [input_node_num, this_node_num], **w_dir)
                b = tf.get_variable('b' + str(layer_id), [1, this_node_num], **b_dir)
                layer = tf.nn.relu(tf.matmul(this_input, w) + b)

            this_input = layer
            input_node_num = this_node_num
            layer_id += 1

        return this_input

    def store_transition(self, observation, action, reward, observation_new):
        """ Store the memory """
        self._reward = reward
        self._reward_hit.append(reward)
        transition = np.hstack((observation, [action[0], action[1], action[2], action[3], reward], observation_new))

        # replace the old memory with new memory 更新记忆池
        index = self._memory_counter % self._memory_size
        self._memory[index, :] = transition

        self._memory_counter += 1

    def choose_action(self, observation):
        """ Choose an action from observation """
        # to have batch dimension when feed into tf placeholder
        observation = observation[np.newaxis, :]

        if np.random.uniform() < self._epsilon:
            # forward feed the observation and get q value for every actions
            actions_value11 = self._sess.run(self._q_eval11, feed_dict={self._s: observation})  # 获取当前环境下所有策略的价值
            action11 = np.argmax(actions_value11)  # 获得最优策略
            actions_value12 = self._sess.run(self._q_eval12, feed_dict={self._s: observation})  # 获取当前环境下所有策略的价值
            action12 = np.argmax(actions_value12)  # 获得最优策略
            actions_value21 = self._sess.run(self._q_eval21, feed_dict={self._s: observation})  # 获取当前环境下所有策略的价值
            action21 = np.argmax(actions_value21)  # 获得最优策略
            actions_value22 = self._sess.run(self._q_eval22, feed_dict={self._s: observation})  # 获取当前环境下所有策略的价值
            action22 = np.argmax(actions_value22)  # 获得最优策略
        else:
            action11 = np.random.randint(0, self._n_actions[0])  # 随机选择一个策略
            action12 = np.random.randint(0, self._n_actions[1])  # 随机选择一个策略
            action21 = np.random.randint(0, self._n_actions[2])  # 随机选择一个策略
            action22 = np.random.randint(0, self._n_actions[3])  # 随机选择一个策略
        return action11, action12, action21, action22

    def learn(self):
        """ Start the learning process """
        # check to replace target parameters
        if self._learn_step_counter % self._replace_target_iter == 0:
            self._sess.run(self._replace_target_op)
            print('\ntarget_params_replaced\n')

        # sample batch memory from all memory 从记忆池抽取batch大小的数据
        if self._memory_counter > self._memory_size:
            sample_index = np.random.choice(self._memory_size, size=self._batch_size)
        else:
            sample_index = np.random.choice(self._memory_counter, size=self._batch_size)
        batch_memory = self._memory[sample_index, :]

        # tar->q_next, eva->q_eval
        q_next11, q_next12, q_next21, q_next22, q_eval11, q_eval12, q_eval21, q_eval22 = self._sess.run(
            [self._q_next11, self._q_next12, self._q_next21, self._q_next22,
             self._q_eval11, self._q_eval12, self._q_eval21, self._q_eval22],
            feed_dict={
                self._s_: batch_memory[:, -self._n_features:],  # fixed params
                self._s: batch_memory[:, :self._n_features],  # newest params
            })

        # change q_target w.r.t q_eval's action
        q_target11 = q_eval11.copy()
        q_target12 = q_eval12.copy()
        q_target21 = q_eval21.copy()
        q_target22 = q_eval22.copy()

        batch_index = np.arange(self._batch_size, dtype=np.int32)
        eval_act_index = batch_memory[:, self._n_features].astype(int)
        reward = batch_memory[:, self._n_features + 1]

        q_target11[batch_index, eval_act_index] = reward + self._gamma * np.max(q_next11, axis=1)
        q_target12[batch_index, eval_act_index] = reward + self._gamma * np.max(q_next12, axis=1)
        q_target21[batch_index, eval_act_index] = reward + self._gamma * np.max(q_next21, axis=1)
        q_target22[batch_index, eval_act_index] = reward + self._gamma * np.max(q_next22, axis=1)

        # train eval network
        _, _, _, _, _cost11, _cost12, _cost21, _cost22, summary = \
            self._sess.run([self._train_op11, self._train_op12, self._train_op21, self._train_op22,
                            self._loss11, self._loss12, self._loss21, self._loss22,
                            self._merged],
                           feed_dict={self._s: batch_memory[:, :self._n_features],
                                      self._q_target11: q_target11,
                                      self._q_target12: q_target12,
                                      self._q_target21: q_target21,
                                      self._q_target22: q_target22})
        self._cost11_his.append(_cost11)
        self._cost12_his.append(_cost12)
        self._cost21_his.append(_cost21)
        self._cost22_his.append(_cost22)

        if self._output_tensorboard:
            self._summary_writer.add_summary(summary, self._learn_step_counter)

        # increasing epsilon
        self._epsilon = (self._epsilon + self._epsilon_increment if self._epsilon < self._epsilon_max
                         else self._epsilon_max)
        self._learn_step_counter += 1

    def plot_cost_reward(self):
        loss1Gif = plt.subplot(2, 2, 1)
        loss1Gif.set_title('loss1')
        loss1Gif.plot(np.arange(len(self._cost11_his)), self._cost11_his)
        loss1Gif.set_ylabel('cost')
        loss1Gif.set_xlabel('steps')
        loss2Gif = plt.subplot(2, 2, 2)
        loss2Gif.set_title('loss2')
        loss2Gif.plot(np.arange(len(self._cost12_his)), self._cost12_his)
        loss2Gif.set_ylabel('cost')
        loss2Gif.set_xlabel('steps')
        loss3Gif = plt.subplot(2, 2, 3)
        loss3Gif.set_title('loss3')
        loss3Gif.plot(np.arange(len(self._cost21_his)), self._cost21_his)
        loss3Gif.set_ylabel('cost')
        loss3Gif.set_xlabel('steps')
        loss4Gif = plt.subplot(2, 2, 4)
        loss4Gif.set_title('loss4')
        loss4Gif.plot(np.arange(len(self._cost22_his)), self._cost22_his)
        loss4Gif.set_ylabel('cost')
        loss4Gif.set_xlabel('steps')
        plt.savefig(os.path.dirname(__file__) + '/data/picture/cost.png')
        plt.close()

        rewardGif = plt.plot(np.arange(len(self._reward_hit)), self._reward_hit)
        rewardGif.set_ylabel('reward')
        rewardGif.set_xlabel('steps')
        plt.savefig(os.path.dirname(__file__) + '/data/picture/reward.png')
        plt.close()

    def save_reward_loss(self, frequency=100):
        tar_path = os.path.dirname(__file__) + '/data/txt/'
        if not os.path.exists(tar_path):
            os.makedirs(tar_path)
        if len(self._reward_hit) > frequency:
            with open(tar_path + 'reward.txt', 'a') as reward_f:
                np.savetxt(reward_f, self._reward_hit)
            with open(tar_path + 'loss1.txt', 'a') as loss11_f:
                np.savetxt(loss11_f, self._cost11_his)
            with open(tar_path + 'loss2.txt', 'a') as loss12_f:
                np.savetxt(loss12_f, self._cost12_his)
            with open(tar_path + 'loss3.txt', 'a') as loss21_f:
                np.savetxt(loss21_f, self._cost21_his)
            with open(tar_path + 'loss4.txt', 'a') as loss22_f:
                np.savetxt(loss22_f, self._cost22_his)

            self._reward_hit.clear()
            self._cost11_his.clear()
            self._cost12_his.clear()
            self._cost21_his.clear()
            self._cost22_his.clear()

    def store_network(self, model_name='MyModel', tar_path=os.path.dirname(__file__) + '/dqnNetModel/'):
        saver = tf.train.Saver()
        saver.save(self._sess, tar_path + model_name)
        self._summary_writer = tf.summary.FileWriter(os.path.dirname(__file__) + "/logs/", self._sess.graph)
        print('save network model successful')

    def load_network(self, tar_path='/dqnNetModel/'):
        saver = tf.train.Saver()
        saver.restore(self._sess, tf.train.latest_checkpoint(os.path.dirname(__file__) + tar_path))
        print('load network model successful')

