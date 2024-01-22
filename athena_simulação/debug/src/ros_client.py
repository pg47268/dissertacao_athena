#! /usr/bin/env python3

import sys
import rospy
import actionlib
import numpy as np
from std_msgs.msg import Float64
from gazebo_msgs.msg import ContactsState, ModelStates, WorldState
from sensor_msgs.msg import JointState, Imu, Range

class Athena:
    def __init__(self, ns='/hexapod/'):
        rospy.init_node('joint_control', anonymous=True)
        self.rate = rospy.Rate(20)

        self.joint_name_lst = None
        self.angles = None

        self._sub_joints = rospy.Subscriber(ns + 'joint_states', JointState, self._cb_joints, queue_size=1)
        rospy.loginfo('Waiting for joints to be populated...')
        while not rospy.is_shutdown():
            if self.joint_name_lst is not None:
                break
            self.rate.sleep()
            rospy.loginfo('Waiting for joints to be populated...')
        rospy.loginfo('Joints populated.')

        rospy.loginfo('Creating joint command publishers.')
        self._pub_joints = {}
        for j in self.joint_name_lst:
            p = rospy.Publisher(ns + j + '_controller/command', Float64, queue_size=1)
            self._pub_joints[j] = p

        self.depth = 0.0
        self.ir = rospy.Subscriber('sensor/ir', Range, self.ir_subscriber, queue_size=1)
        self.orientation, self.ang_v, self.accel = np.zeros(4, dtype=float), np.zeros(3, dtype=float), np.zeros(3, dtype=float)
        self.imu_subscriber = rospy.Subscriber(ns + 'imu', Imu, self.imu_subscriber_callback, queue_size=1)
        self._posicao = []
        self.model_subscriber = rospy.Subscriber('gazebo/model_states', ModelStates, self.model_state_callback)
        self.sim_time = 0.0
        self.tempo_subscriber = rospy.Subscriber("/gazebo/world_state", WorldState, self.world_state_callback)
        # Sonar sensors
        self.sonar = {}
        self.sonar_1 = rospy.Subscriber('sensor/sonar_1', Range, self.sonar_1_callback, queue_size=1)
        self.sonar_2 = rospy.Subscriber('sensor/sonar_2', Range, self.sonar_2_callback, queue_size=1)
        self.sonar_3 = rospy.Subscriber('sensor/sonar_3', Range, self.sonar_3_callback, queue_size=1)
        self.sonar_4 = rospy.Subscriber('sensor/sonar_4', Range, self.sonar_4_callback, queue_size=1)
        self.sonar_5 = rospy.Subscriber('sensor/sonar_5', Range, self.sonar_5_callback, queue_size=1)
        self.sonar_6 = rospy.Subscriber('sensor/sonar_6', Range, self.sonar_6_callback, queue_size=1)
        self.sonar_7 = rospy.Subscriber('sensor/sonar_7', Range, self.sonar_7_callback, queue_size=1)
        self.sonar_8 = rospy.Subscriber('sensor/sonar_8', Range, self.sonar_8_callback, queue_size=1)

        # Contact forces
        self.force = {}
        self.foot_1 = rospy.Subscriber('/foot_1_bumper', ContactsState, self.foot_1_callback, queue_size=1)
        self.foot_2 = rospy.Subscriber('/foot_2_bumper', ContactsState, self.foot_2_callback, queue_size=1)
        self.foot_3 = rospy.Subscriber('/foot_3_bumper', ContactsState, self.foot_3_callback, queue_size=1)
        self.foot_4 = rospy.Subscriber('/foot_4_bumper', ContactsState, self.foot_4_callback, queue_size=1)
        self.foot_5 = rospy.Subscriber('/foot_5_bumper', ContactsState, self.foot_5_callback, queue_size=1)
        self.foot_6 = rospy.Subscriber('/foot_6_bumper', ContactsState, self.foot_6_callback, queue_size=1)
        rospy.sleep(1)
        rospy.loginfo('ROS-Client set. Ready to walk.')

    def _cb_joints(self, msg):
        if self.joint_name_lst is None:
            self.joint_name_lst = msg.name
        self.angles = msg.position

    def get_angles(self):
        if self.joint_name_lst is None:
            return None
        if self.angles is None:
            return None
        return dict(zip(self.joint_name_lst, self.angles))

    def set_angles(self, angles):
        #rospy.loginfo('Publishing joints...')
        for j,v in angles.items():
            if j not in self.joint_name_lst:
                rospy.logerror('Invalid joint name "'+ j +'"')
                continue
            self._pub_joints[j].publish(v)

    def imu_subscriber_callback(self, imu):
        self.orientation = np.array([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
        self.ang_v = np.array([imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z])
        self.accel = np.array([imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z])

    def ir_subscriber(self, ir):
        self.depth = ir.range

    def model_state_callback(self, posicao):
        model_index = posicao.name.index('hexapod')
        self._posicao = posicao.pose[model_index].position

    def world_state_callback(self, world_state):
        current_sim_time = world_state.sim_time
        self.sim_time = current_sim_time.to_sec()

    # SUBSCRIBERS FOR THE SONARS
    def sonar_1_callback(self, sensor):
        self.sonar[0] = sensor.range
    def sonar_2_callback(self, sensor):
        self.sonar[1] = sensor.range
    def sonar_3_callback(self, sensor):
        self.sonar[2] = sensor.range
    def sonar_4_callback(self, sensor):
        self.sonar[3] = sensor.range
    def sonar_5_callback(self, sensor):
        self.sonar[4] = sensor.range
    def sonar_6_callback(self, sensor):
        self.sonar[5] = sensor.range
    def sonar_7_callback(self, sensor):
        self.sonar[6] = sensor.range
    def sonar_8_callback(self, sensor):
        self.sonar[7] = sensor.range

    # SUBSCRIBERS FOR THE CONTACT FORCES
    def foot_1_callback(self, bumper):
        if len(bumper.states) >= 1:
            self.force[0] = {'id': 1, 'contact': True, 'normal_force': bumper.states[0].total_wrench.force.z}
        else:
            self.force[0] = {'id': 1, 'contact': False, 'normal_force': 0.0}

    def foot_2_callback(self, bumper):
        if len(bumper.states) >= 1:
            self.force[1] = {'id': 2, 'contact': True, 'normal_force': bumper.states[0].total_wrench.force.z}
        else:
            self.force[1] = {'id': 2, 'contact': False, 'normal_force': 0.0}

    def foot_3_callback(self, bumper):
        if len(bumper.states) >= 1:
            self.force[2] = {'id': 3, 'contact': True, 'normal_force': bumper.states[0].total_wrench.force.z}
        else:
            self.force[2] = {'id': 3, 'contact': False, 'normal_force': 0.0}

    def foot_4_callback(self, bumper):
        if len(bumper.states) >= 1:
            self.force[3] = {'id': 4, 'contact': True, 'normal_force': bumper.states[0].total_wrench.force.z}
        else:
            self.force[3] = {'id': 4, 'contact': False, 'normal_force': 0.0}

    def foot_5_callback(self, bumper):
        if len(bumper.states) >= 1:
            self.force[4] = {'id': 5, 'contact': True, 'normal_force': bumper.states[0].total_wrench.force.z}
        else:
            self.force[4] = {'id': 5, 'contact': False, 'normal_force': 0.0}

    def foot_6_callback(self, bumper):
        if len(bumper.states) >= 1:
            self.force[5] = {'id': 6, 'contact': True, 'normal_force': bumper.states[0].total_wrench.force.z}
        else:
            self.force[5] = {'id': 6, 'contact': False, 'normal_force': 0.0}
