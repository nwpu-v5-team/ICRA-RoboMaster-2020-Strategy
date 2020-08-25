from collections import deque

import numpy
import rospy

from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from roborts_msgs.msg import BuffZoneStatus
from roborts_msgs.msg import GameStatus
from roborts_msgs.msg import GameSurvivor
from roborts_msgs.msg import RobotStatus
from roborts_msgs.msg import RobotStrategyState
from roborts_msgs.msg import ShootInfo

from std_msgs.msg import Bool
from std_msgs.msg import Int32

from openai_ros import robot_gazebo_env
from util import init_from_dict, onehot_encode, quart_to_rpy, init_quaternion


class FireflyEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """
    def __init__(self):
        """
        Initializes a new FireflyEnv environment.

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.

        The Sensors: The sensors accesible are the ones considered usefull for AI learning.

        Args:
        """
        rospy.logdebug("Start FireflyEnv INIT...")

        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(FireflyEnv, self).__init__(controllers_list=self.controllers_list,
                                         robot_name_space=self.robot_name_space,
                                         reset_controls=False,
                                         start_init_physics_parameters=False,
                                         reset_world_or_sim="WORLD")
        self.gazebo.unpauseSim()

        self._check_all_sensors_ready()

        self.red1_ground_truth_pose = Odometry()
        self.red2_ground_truth_pose = Odometry()
        self.blue1_ground_truth_pose = Odometry()
        self.blue2_ground_truth_pose = Odometry()

        self.red1_shoot_info = ShootInfo()
        self.red2_shoot_info = ShootInfo()
        self.blue1_shoot_info = ShootInfo()
        self.blue2_shoot_info = ShootInfo()

        self.red1_robot_status = RobotStatus()
        self.red2_robot_status = RobotStatus()
        self.blue1_robot_status = RobotStatus()
        self.blue2_robot_status = RobotStatus()

        self.buff_zone_status = BuffZoneStatus()
        self.game_status = GameStatus()
        self.game_survivor = GameSurvivor()

        self._robot_strategy_status = RobotStrategyState()

        self._my_color = rospy.get_param('my_color')

        # self.gazebo.unpauseSim()
        # self.controllers_object.reset_controllers()

        # We Start all the ROS related Subscribers and publishers

        # ground_truth/state Odometry
        rospy.Subscriber("/red2/ground_truth/state", Odometry, self._red2_ground_truth_callback)
        rospy.Subscriber("/red1/ground_truth/state", Odometry, self._red1_ground_truth_callback)
        rospy.Subscriber("/blue2/ground_truth/state", Odometry, self._blue2_ground_truth_callback)
        rospy.Subscriber("/blue1/ground_truth/state", Odometry, self._blue1_ground_truth_callback)

        # shoot_info ShootInfo
        rospy.Subscriber("/red2/shoot_info", ShootInfo, self._red2_shoot_info_callback)
        rospy.Subscriber("/red1/shoot_info", ShootInfo, self._red1_shoot_info_callback)
        rospy.Subscriber("/blue2/shoot_info", ShootInfo, self._blue2_shoot_info_callback)
        rospy.Subscriber("/blue1/shoot_info", ShootInfo, self._blue1_shoot_info_callback)

        # robot_status RobotStatus
        rospy.Subscriber("/red2/robot_status", RobotStatus, self._red2_robot_status_callback)
        rospy.Subscriber("/red1/robot_status", RobotStatus, self._red1_robot_status_callback)
        rospy.Subscriber("/blue2/robot_status", RobotStatus, self._blue2_robot_status_callback)
        rospy.Subscriber("/blue1/robot_status", RobotStatus, self._blue1_robot_status_callback)

        # /buff_zone_status
        rospy.Subscriber("/buff_zone_status", BuffZoneStatus, self._buff_zone_status_callback)

        # /game_status
        rospy.Subscriber("/game_status", GameStatus, self._game_status_callback)

        # /game_survivor
        rospy.Subscriber("/game_survivor", GameSurvivor, self._game_survivor_callback)

        rospy.Subscriber("/is_collide", RobotStrategyState, self._robot_strategy_status_callback)

        self._acion_id_pub = rospy.Publisher('/' + self._my_color + '/action_id', Int32, queue_size=1)

        self._robot_pose_pub = rospy.Publisher('/gazebo/set_model_states', ModelState, queue_size=1)

        self._reset_referee_pub = rospy.Publisher('/reset_referee', Bool, queue_size=1)

        # self._check_publishers_connection()

        # Test PauseSim
        self.gazebo.pauseSim()

        rospy.logdebug("Finished FireflyEnv INIT...")

        self.rate = rospy.Rate(2)

        self.my_robots_pose = [deque(maxlen=200), deque(maxlen=200)]

    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------

    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")

        # New add the check robots ground truth
        self._check_red2_ground_truth_ready()
        self._check_red1_ground_truth_ready()
        self._check_blue1_ground_truth_ready()
        self._check_blue2_ground_truth_ready()

        # New add the check robots status
        self._check_red1_robot_status_ready()
        self._check_red2_robot_status_ready()
        self._check_blue1_robot_status_ready()
        self._check_blue2_robot_status_ready()

        # New add the check robots shoot info
        self._check_red1_shoot_info_ready()
        self._check_red2_shoot_info_ready()
        self._check_blue1_shoot_info_ready()
        self._check_blue2_shoot_info_ready()

        # New add the check game status
        self._check_game_status_ready()
        self._check_game_survivor_ready()
        self._check_buff_zone_status_ready()

        rospy.logdebug("ALL SENSORS READY")

    # check ground_truth ready
    def _check_red2_ground_truth_ready(self):
        self.red2_ground_truth_pose = None
        rospy.logdebug("Waiting for /red2/ground_truth/state to be READY...")
        while self.red2_ground_truth_pose is None and not rospy.is_shutdown():
            try:
                self.red2_ground_truth_pose = rospy.wait_for_message("/red2/ground_truth/state", Odometry, timeout=1.0)
                rospy.logdebug("Current /red2/ground_truth/state READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /red2/ground_truth/state not ready yet, retrying for getting red2 pose")

        return self.red2_ground_truth_pose

    def _check_red1_ground_truth_ready(self):
        self.red1_ground_truth_pose = None
        rospy.logdebug("Waiting for /red1/ground_truth/state to be READY...")
        while self.red1_ground_truth_pose is None and not rospy.is_shutdown():
            try:
                self.red1_ground_truth_pose = rospy.wait_for_message("/red1/ground_truth/state", Odometry, timeout=1.0)
                rospy.logdebug("Current /red1/ground_truth/state READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /red1/ground_truth/state not ready yet, retrying for getting red1 pose")

        return self.red1_ground_truth_pose

    def _check_blue1_ground_truth_ready(self):
        self.blue1_ground_truth_pose = None
        rospy.logdebug("Waiting for /blue1/ground_truth/state to be READY...")
        while self.blue1_ground_truth_pose is None and not rospy.is_shutdown():
            try:
                self.blue1_ground_truth_pose = rospy.wait_for_message("/blue1/ground_truth/state", Odometry,
                                                                      timeout=1.0)
                rospy.logdebug("Current /blue1/ground_truth/state READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /blue1/ground_truth/state not ready yet, retrying for getting blue1 pose")

        return self.blue1_ground_truth_pose

    def _check_blue2_ground_truth_ready(self):
        self.blue2_ground_truth_pose = None
        rospy.logdebug("Waiting for /blue2/ground_truth/state to be READY...")
        while self.blue2_ground_truth_pose is None and not rospy.is_shutdown():
            try:
                self.blue2_ground_truth_pose = rospy.wait_for_message("/blue2/ground_truth/state", Odometry,
                                                                      timeout=1.0)
                rospy.logdebug("Current /blue2/ground_truth/state READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /blue2/ground_truth/state not ready yet, retrying for getting blue2 pose")

        return self.blue2_ground_truth_pose

    # check robot_status ready
    def _check_red2_robot_status_ready(self):
        self.red2_robot_status = None
        rospy.logdebug("Waiting for /red2/robot_status to be READY...")
        while self.red2_robot_status is None and not rospy.is_shutdown():
            try:
                self.red2_robot_status = rospy.wait_for_message("/red2/robot_status", RobotStatus, timeout=1.0)
                rospy.logdebug("Current /red2/robot_status READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /red2/robot_status not ready yet, retrying for getting red2 robot_status")

        return self.red2_robot_status

    def _check_red1_robot_status_ready(self):
        self.red1_robot_status = None
        rospy.logdebug("Waiting for /red1/robot_status to be READY...")
        while self.red1_robot_status is None and not rospy.is_shutdown():
            try:
                self.red1_robot_status = rospy.wait_for_message("/red1/robot_status", RobotStatus, timeout=1.0)
                rospy.logdebug("Current /red1/robot_status READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /red1/robot_status not ready yet, retrying for getting red1 robot_status")

        return self.red1_robot_status

    def _check_blue1_robot_status_ready(self):
        self.blue1_robot_status = None
        rospy.logdebug("Waiting for /blue1/robot_status to be READY...")
        while self.blue1_robot_status is None and not rospy.is_shutdown():
            try:
                self.blue1_robot_status = rospy.wait_for_message("/blue1/robot_status", RobotStatus, timeout=1.0)
                rospy.logdebug("Current /blue1/robot_status READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /blue1/robot_status not ready yet, retrying for getting blue1 robot_status")

        return self.blue1_robot_status

    def _check_blue2_robot_status_ready(self):
        self.blue2_robot_status = None
        rospy.logdebug("Waiting for /blue2/robot_status to be READY...")
        while self.blue2_robot_status is None and not rospy.is_shutdown():
            try:
                self.blue2_robot_status = rospy.wait_for_message("/blue2/robot_status", RobotStatus, timeout=1.0)
                rospy.logdebug("Current /blue2/robot_status READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /blue2/robot_status not ready yet, retrying for getting blue2 robot_status")

        return self.blue2_robot_status

    # Robot ground truth callback
    def _red2_ground_truth_callback(self, data: Odometry):
        self.red2_ground_truth_pose = data

    def _red1_ground_truth_callback(self, data: Odometry):
        self.red1_ground_truth_pose = data

    def _blue1_ground_truth_callback(self, data: Odometry):
        self.blue1_ground_truth_pose = data

    def _blue2_ground_truth_callback(self, data: Odometry):
        self.blue2_ground_truth_pose = data

    # Robot shoot info callback
    def _red2_shoot_info_callback(self, data):
        self.red2_shoot_info = data

    def _red1_shoot_info_callback(self, data):
        self.red1_shoot_info = data

    def _blue1_shoot_info_callback(self, data):
        self.blue1_shoot_info = data

    def _blue2_shoot_info_callback(self, data):
        self.blue2_shoot_info = data

    # Check the robot shoot info
    def _check_red2_shoot_info_ready(self):
        self.red2_shoot_info = None
        rospy.logdebug("Waiting for /red2/shoot_info to be READY...")
        while self.red2_shoot_info is None and not rospy.is_shutdown():
            try:
                self.red2_shoot_info = rospy.wait_for_message("/red2/shoot_info", ShootInfo, timeout=1.0)
                rospy.logdebug("Current /red2/shoot_info READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /red2/shoot_info not ready yet, retrying for getting red2 shoot_info")

        return self.red2_shoot_info

    def _check_red1_shoot_info_ready(self):
        self.red1_shoot_info = None
        rospy.logdebug("Waiting for /red1/shoot_info to be READY...")
        while self.red1_shoot_info is None and not rospy.is_shutdown():
            try:
                self.red1_shoot_info = rospy.wait_for_message("/red1/shoot_info", ShootInfo, timeout=1.0)
                rospy.logdebug("Current /red1/shoot_info READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /red1/shoot_info not ready yet, retrying for getting red1 shoot_info")

        return self.red1_shoot_info

    def _check_blue1_shoot_info_ready(self):
        self.blue1_shoot_info = None
        rospy.logdebug("Waiting for /blue1/shoot_info to be READY...")
        while self.blue1_shoot_info is None and not rospy.is_shutdown():
            try:
                self.blue1_shoot_info = rospy.wait_for_message("/blue1/shoot_info", ShootInfo, timeout=1.0)
                rospy.logdebug("Current /blue1/shoot_info READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /blue1/shoot_info not ready yet, retrying for getting blue1 shoot_info")
        return self.blue1_shoot_info

    def _check_blue2_shoot_info_ready(self):
        self.blue2_shoot_info = None
        rospy.logdebug("Waiting for /blue2/shoot_info to be READY...")
        while self.blue2_shoot_info is None and not rospy.is_shutdown():
            try:
                self.blue2_shoot_info = rospy.wait_for_message("/blue2/shoot_info", ShootInfo, timeout=1.0)
                rospy.logdebug("Current /blue2/shoot_info READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /blue2/shoot_info not ready yet, retrying for getting blue2 shoot_info")
        return self.blue2_shoot_info

    # callback / robot_status_callback
    def _red2_robot_status_callback(self, data):
        self.red2_robot_status = data

    def _red1_robot_status_callback(self, data):
        self.red1_robot_status = data

    def _blue1_robot_status_callback(self, data):
        self.blue1_robot_status = data

    def _blue2_robot_status_callback(self, data):
        self.blue2_robot_status = data

    # callback /buff_zone_status
    def _buff_zone_status_callback(self, data):
        self.buff_zone_status = data

    # check the buff zone status
    def _check_buff_zone_status_ready(self):
        self.buff_zone_status = None
        rospy.logdebug("Waiting for //buff_zone_status to be READY...")
        while self.buff_zone_status is None and not rospy.is_shutdown():
            try:
                self.buff_zone_status = rospy.wait_for_message("/buff_zone_status", BuffZoneStatus, timeout=1.0)
                rospy.logdebug("Current /buff_zone_status READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /buff_zone_status not ready yet, retrying for getting buff_zone_status")

        return self.buff_zone_status

    # callback /game_status
    def _game_status_callback(self, data):
        self.game_status = data

    # check the game status
    def _check_game_status_ready(self):
        self.game_status = None
        rospy.logdebug("Waiting for /game_status to be READY...")
        while self.game_status is None and not rospy.is_shutdown():
            try:
                self.game_status = rospy.wait_for_message("/game_status", GameStatus, timeout=1.0)
                rospy.logdebug("Current /game_status READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /game_status not ready yet, retrying for getting game_status")

        return self.game_status

    # callback /game_survivor
    def _game_survivor_callback(self, data):
        self.game_survivor = data

    # check the game survivor
    def _check_game_survivor_ready(self):
        self.game_survivor = None
        rospy.logdebug("Waiting for /game_survivor to be READY...")
        while self.game_survivor is None and not rospy.is_shutdown():
            try:
                self.game_survivor = rospy.wait_for_message("/game_survivor", GameSurvivor, timeout=1.0)
                rospy.logdebug("Current /game_survivor READY=>")
            except rospy.ROSException:
                rospy.logerr("Current /game_survivor not ready yet, retrying for getting game_survivor")

        return self.game_survivor

    def _robot_strategy_status_callback(self, data: RobotStrategyState):
        self._robot_strategy_status = data

    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self._acion_id_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to _action_id_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug(" _cmd_vel_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        red1 = ModelState()
        init_from_dict(red1, {'model_name': 'red1',
                              'pose': {'position': {'x': 3.486985, 'y': -1.655670, 'z': 0},
                                       'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}},
                              'twist': Twist(),
                              'reference_frame': 'map'})
        self._robot_pose_pub.publish(red1)

        red2 = ModelState()
        init_from_dict(red2, {'model_name': 'red2',
                              'pose': {'position': {'x': 3.486985, 'y': 1.655670, 'z': 0},
                                       'orientation': {'x': 0, 'y': 0, 'z': 0.706825181105, 'w': 0.707388269167}},
                              'twist': Twist(),
                              'reference_frame': 'map'})

        self._robot_pose_pub.publish(red2)

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        msg = Bool()
        msg.data = True
        self._reset_referee_pub.publish(msg)
        pass

    def _env_setup(self, initial_qpos):
        pass

    def render(self, mode='human'):
        pass

    def _compute_reward(self, observations, done, my_color='red'):
        """Calculates the reward to give based on the observations given.
        我方血量：+k1×血量/2000
        敌方血量：-k1×血量/2000
        敌方死：+MAX
        我方死：-MAX
        敌方子弹量：-k2×敌方子弹/100
        我方子弹量：+k2×我方子弹/100
        执行上一拍策略：2^(-x)  连续执行次数x
        """
        K1 = 1.0 / 2000
        K2 = 0.01 / 100
        MAX = 10.0

        red1, red2 = self.get_red1_robot_status(), self.get_red2_robot_status()
        blue1, blue2 = self.get_blue1_robot_status(), self.get_blue2_robot_status()
        red1_shoot, red2_shoot = self.get_red1_shoot_info(), self.get_red2_shoot_info()
        blue1_shoot, blue2_shoot = self.get_blue1_shoot_info(), self.get_blue2_shoot_info()
        red1_pos, red2_pos = self.get_red1_ground_truth(), self.get_red2_ground_truth()
        blue1_pos, blue2_pos = self.get_blue1_ground_truth(), self.get_blue2_ground_truth()

        if my_color == 'red':
            my1, my2, they1, they2 = red1, red2, blue1, blue2
            my1_sh, my2_sh, they1_sh, they2_sh = red1_shoot, red2_shoot, blue1_shoot, blue2_shoot
            my1_pos, my2_pos = red1_pos, red2_pos
        elif my_color == 'blue':
            my1, my2, they1, they2 = blue1, blue2, red1, red2
            my1_sh, my2_sh, they1_sh, they2_sh = blue1_shoot, blue2_shoot, red1_shoot, red2_shoot
            my1_pos, my2_pos = blue1_pos, blue2_pos
        else:
            rospy.logerr('I don\'t know my color!!!')
            return 0

        self.my_robots_pose[0].append(my1_pos.pose.pose.position)
        self.my_robots_pose[1].append(my2_pos.pose.pose.position)

        reward = 0.0
        reward += K1 * (my1.remain_hp + my2.remain_hp - they1.remain_hp - they2.remain_hp)
        reward += K2 * (my1_sh.sent_bullet + my2_sh.sent_bullet - they1_sh.sent_bullet - they2_sh.sent_bullet)
        reward += MAX if they1.remain_hp == 0 else 0.0
        reward += MAX if they2.remain_hp == 0 else 0.0
        reward += MAX if my1.remain_hp == 0 else 0.0
        reward += MAX if my2.remain_hp == 0 else 0.0
        reward += MAX * self.is_roll_over()
        reward += MAX * self._robot_strategy_status.my_1_is_collide
        reward += MAX * self._robot_strategy_status.my_2_is_collide
        return reward

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        self._acion_id_pub.publish(action)
        self.rate.sleep()

    def _get_obs(self):

        robot_status = [self.red1_robot_status, self.red2_robot_status, self.blue1_robot_status,
                        self.blue2_robot_status]
        shoot_info = [self.red1_shoot_info, self.red2_shoot_info, self.blue1_shoot_info, self.blue2_shoot_info]
        robot_pose = [self.red1_ground_truth_pose, self.red2_ground_truth_pose, self.blue1_ground_truth_pose,
                      self.blue2_ground_truth_pose]

        observations = []

        for status in robot_status:
            observations.extend(
                [status.remain_hp, status.level, status.shooter_output, status.gimbal_output, status.chassis_output])

        for shoot in shoot_info:
            observations.extend([shoot.remain_bullet, shoot.sent_bullet, shoot.fric_wheel_run])

        for pose in robot_pose:
            position = pose.pose.pose.position
            orientation = pose.pose.pose.orientation
            observations.extend([position.x, position.y, orientation.x, orientation.y, orientation.z, orientation.w])

        # 0 - 7, onehot encoding
        buff_states = [self.buff_zone_status.F1_zone_buff_debuff_status * self.buff_zone_status.F1_zone_status,
                       self.buff_zone_status.F2_zone_buff_debuff_status * self.buff_zone_status.F2_zone_status,
                       self.buff_zone_status.F3_zone_buff_debuff_status * self.buff_zone_status.F3_zone_status,
                       self.buff_zone_status.F4_zone_buff_debuff_status * self.buff_zone_status.F4_zone_status,
                       self.buff_zone_status.F5_zone_buff_debuff_status * self.buff_zone_status.F5_zone_status,
                       self.buff_zone_status.F6_zone_buff_debuff_status * self.buff_zone_status.F6_zone_status,
                       ]
        for buff_state in map(lambda x: onehot_encode(x, 8), buff_states):
            observations.extend(buff_state)

        observations.append(self.game_status.remaining_time)

        observations.extend([self.game_survivor.red3,
                             self.game_survivor.red4,
                             self.game_survivor.blue3,
                             self.game_survivor.blue4])

        return numpy.array(observations)

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        if self.get_blue1_robot_status().remain_hp <= 0 and self.get_blue2_robot_status().remain_hp <= 0:
            print('blue is die')
            return True
        elif self.get_red1_robot_status().remain_hp <= 0 and self.get_red2_robot_status().remain_hp <= 0:
            print('red is die')
            return True
        elif self.is_roll_over():
            print("robot is roll over!")
            return True
        elif self._is_stop():
            print('robot is stop for a long time')
            return True
        elif self._robot_strategy_status.my_1_is_collide or self._robot_strategy_status.my_2_is_collide:
            print('our robot is collide')
            return True
        return False

    def start_game(self):
        red1 = ModelState()
        init_from_dict(red1, {'model_name': 'red1',
                              'pose': {'position': {'x': 0, 'y': -1.655670, 'z': 0},
                                       'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}},
                              'twist': Twist(),
                              'reference_frame': 'map'})
        self._robot_pose_pub.publish(red1)

        red2 = ModelState()
        init_from_dict(red2, {'model_name': 'red2',
                              'pose': {'position': {'x': 3.486985, 'y': 1.655670, 'z': 0},
                                       'orientation': {'x': 0, 'y': 0, 'z': 0.706825181105, 'w': 0.707388269167}},
                              'twist': Twist(),
                              'reference_frame': 'map'})

        self._robot_pose_pub.publish(red2)

        blue1 = ModelState()
        init_from_dict(blue1, {'model_name': 'blue1',
                               'pose': {'position': {'x': -3.486985, 'y': 1.65567, 'z': 0},
                                        'orientation': {'x': 0, 'y': 0, 'z': 0.999999999999,
                                                        'w': 1.32679489668e-06}},
                               'twist': Twist(),
                               'reference_frame': 'map'})

        self._robot_pose_pub.publish(blue1)

        blue2 = ModelState()
        init_from_dict(blue2, {'model_name': 'blue2',
                               'pose': {'position': {'x': -3.486985, 'y': -1.65567, 'z': 0},
                                        'orientation': {'x': 0, 'y': 0, 'z': -0.706825181105,
                                                        'w': 0.707388269167}},
                               'twist': Twist(),
                               'reference_frame': 'map'})

        self._robot_pose_pub.publish(blue2)

    # Get the ground truth
    def get_blue1_ground_truth(self):
        return self.blue1_ground_truth_pose

    def get_blue2_ground_truth(self):
        return self.blue2_ground_truth_pose

    def get_red1_ground_truth(self):
        return self.red1_ground_truth_pose

    def get_red2_ground_truth(self):
        return self.red2_ground_truth_pose

    # Get the shoot info
    def get_red2_shoot_info(self) -> ShootInfo:
        return self.red2_shoot_info

    def get_red1_shoot_info(self) -> ShootInfo:
        return self.red1_shoot_info

    def get_blue1_shoot_info(self) -> ShootInfo:
        return self.blue1_shoot_info

    def get_blue2_shoot_info(self) -> ShootInfo:
        return self.blue2_shoot_info

    # Get the robot status
    def get_blue1_robot_status(self) -> RobotStatus:
        return self.blue1_robot_status

    def get_blue2_robot_status(self) -> RobotStatus:
        return self.blue2_robot_status

    def get_red1_robot_status(self) -> RobotStatus:
        return self.red1_robot_status

    def get_red2_robot_status(self) -> RobotStatus:
        return self.red2_robot_status

    # Get the buff zones status
    def get_buff_zones_status(self):
        return self.buff_zone_status

    # Get the game status
    def get_game_status(self):
        return self.game_status

    # Get the game survivor
    def get_game_survivor(self):
        return self.game_survivor

    def reinit_sensors(self):
        """
        This method is for the tasks so that when reseting the episode
        the sensors values are forced to be updated with the real data and

        """

    # Test
    def get_obs(self):
        return self._get_obs()

    @staticmethod
    def _get_continuous_data_num(data_list: list, finish=-1):
        """
        获得从finish向前连续的数据个数
        :param finish: 计数的开始位置（从后往前）
        :return: 连续的数据个数
        """
        data_continuous_number = 0
        for i in range(-finish, len(data_list) + 1):
            if abs(data_list[-i] - data_list[finish]) < 0.01:
                data_continuous_number += 1
            pass
        return data_continuous_number

    def is_roll_over(self):
        r1_quaternion = init_quaternion(self.red1_ground_truth_pose.pose.pose.orientation)
        r1_roll, r1_pitch, r1_yaw = quart_to_rpy(*r1_quaternion)
        r2_quaternion = init_quaternion(self.red2_ground_truth_pose.pose.pose.orientation)
        r2_roll, r2_pitch, r2_yaw = quart_to_rpy(*r2_quaternion)

        b1_quaternion = init_quaternion(self.blue1_ground_truth_pose.pose.pose.orientation)
        b1_roll, b1_pitch, b1_yaw = quart_to_rpy(*b1_quaternion)
        b2_quaternion = init_quaternion(self.blue2_ground_truth_pose.pose.pose.orientation)
        b2_roll, b2_pitch, b2_yaw = quart_to_rpy(*b2_quaternion)
        if abs(r1_roll) > 0.15 or abs(r1_pitch) > 0.15 \
                or abs(r2_roll) > 0.15 or abs(r2_pitch) > 0.15 \
                or abs(b1_roll) > 0.15 or abs(b1_pitch) > 0.15 \
                or abs(b2_roll) > 0.15 or abs(b2_pitch) > 0.15:
            return True
        else:
            return False

    def _is_stop(self, stop_time=60):
        """
        判断若我方两个机器人是否长时间静止
        :return:
        """
        if FireflyEnv._get_continuous_data_num([i.x for i in self.my_robots_pose[0]]) > stop_time \
                and FireflyEnv._get_continuous_data_num([i.y for i in self.my_robots_pose[0]]) > stop_time:
            return True
        elif FireflyEnv._get_continuous_data_num([i.x for i in self.my_robots_pose[1]]) > stop_time \
                and FireflyEnv._get_continuous_data_num([i.y for i in self.my_robots_pose[1]]) > stop_time:
            return True
        return False
