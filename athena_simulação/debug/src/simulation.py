#! /usr/bin/env python3

import os
import sys
import json
import time
import rospy
import signal
import subprocess
import numpy as np
import pandas as pd
from ros_client import Athena
from std_srvs.srv import Empty
import matplotlib.pyplot as plt
from kinematics import Kinematics
from gazebo_msgs.msg import ModelState
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest

class WalkFunction:
    def __init__(self):
        self.height = 0.3
        self.pos = np.zeros((6, 4), dtype=float)
        self.p0 = np.zeros((6, 4), dtype=float)
        self._tf = 1.0

    def p1_p(self, p0, id):
        # Requires the angular positions
        #p0 = Kinematics(id).fk(joints)

        s = 0.045
        t1 = np.array([0, 0, 0])
        t2 = np.array([0, -0.00032, 0.00384])
        t3 = np.array([0, s, 0, 0])

        # Control points
        p1 = t1 + p0
        p2 = t2 + p0
        pf = t3 + p0

        parameters = {'p0': p0, 'p1': p1, 'p2': p2, 'p3': pf}
        return parameters

    def p2_p(self, p0, id):
        # Requires the angular positions
        #p0 = Kinematics(id).fk(joints)

        s = 0.045
        t1 = np.array([0, s/4, self.height/3, 0])
        t2 = np.array([0, (3/4)*s, self.height/3, 0])
        t3 = np.array([0, s, 0, 0])

        parameters = {'p0': p0, 't1': t1, 't2': t2, 't3': t3}
        return parameters

    def p1(self, params, t, contact):
        id = params.get('id')
        p0 = np.array(params.get('p0'))
        p1 = np.array(params.get('p1'))
        p2 = np.array(params.get('p2'))
        p3 = np.array(params.get('p3'))

        if t <= self._tf:
            if t >= 1/5 * self._tf and contact is True:
                return Kinematics(id).ik(self.pos[id])
            else:
                self.pos[id] = np.power(1 - t, 3) * p0 + 3 * t * np.power((1 - t), 2) * p1 + 3 * np.power(t, 2) * (1 - t) * p2 + np.power(t, 3) * p3
                return Kinematics(id).ik(self.pos[id])
        else:
            s = -0.045
            self.p0[id] = np.array([self.pos[id][0], self.pos[id][1] + (s/1.0) * (t - 1.0), self.pos[id][2], 1])
            return Kinematics(id).ik(self.p0[id])

    def p2(self, params, t, contact):
        id = params.get('id')
        p0 = params.get('p0')
        t1 = np.array(params.get('t1'))
        t2 = np.array(params.get('t2'))
        t3 = np.array(params.get('t3'))

        if t <= self._tf:
            s = -0.045
            self.p0[id] = np.array([p0[0], p0[1] + (s/self._tf) * t, p0[2], 1])
            return Kinematics(id).ik(self.p0[id])
        else:
            # Control points
            p1 = t1 + self.p0[id]
            p2 = t2 + self.p0[id]
            p3 = t3 + self.p0[id]
            if (t - self._tf) >= 1/5 * self._tf and contact is True:
                return Kinematics(id).ik(self.pos[id])
            else:
                self.pos[id] = np.power(2.0 - t, 3) * self.p0[id] + 3 * (t - 1.0) * np.power((2.0 - t), 2) * p1 + 3 * np.power(t - 1.0, 2) * (2.0 - t) * p2 + np.power(t - 1.0, 3) * p3
                #self.pos[id] = np.power(1 - (t - self._tf), 3) * self.p0[id] + 3 * t * np.power((1 - (t - self._tf)), 2) * p1 + 3 * np.power(t - self._tf, 2) * (1 - (t - self._tf)) * p2 + np.power(t - self._tf, 3) * p3
                return Kinematics(id).ik(self.pos[id])
        

class Env:
    def __init__(self):
        # ROS Client
        self.agent = Athena()
        self.joints = self.agent.joint_name_lst
        self.jp, self.error = {}, {}
        for joint in self.joints:
            self.jp[joint] = 0.0
            self.error[joint] = 0.0

        self._time = 2.0
        #self._time = np.arange(0, 1.05, 50e-3)
        self.walker = WalkFunction()

        # Servo Control
        self.k = 0.598

        self.l1, self.l2, self.l3, self.l4, self.l5, self.l6 = np.empty([1, 3]), np.empty([1, 3]), np.empty([1, 3]), np.empty([1, 3]), np.empty([1, 3]), np.empty([1, 3])
        self.a1, self.a2, self.a3, self.a4, self.a5, self.a6 = np.empty([1, 3]), np.empty([1, 3]), np.empty([1, 3]), np.empty([1, 3]), np.empty([1, 3]), np.empty([1, 3])

        # Gazebo shenanigans
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        self.model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_state_req = SetModelStateRequest()
        self.model_state_req.model_state = ModelState()
        self.model_state_req.model_state.model_name = 'hexapod'
        self.model_state_req.model_state.pose.position.x = 0.0
        self.model_state_req.model_state.pose.position.y = 0.0
        self.model_state_req.model_state.pose.position.z = 0.0
        self.model_state_req.model_state.pose.orientation.x = 0.0
        self.model_state_req.model_state.pose.orientation.y = 0.0
        self.model_state_req.model_state.pose.orientation.z = 0.0
        self.model_state_req.model_state.pose.orientation.w = 0.0
        self.model_state_req.model_state.twist.linear.x = 0.0
        self.model_state_req.model_state.twist.linear.y = 0.0
        self.model_state_req.model_state.twist.linear.z = 0.0
        self.model_state_req.model_state.twist.angular.x = 0.0
        self.model_state_req.model_state.twist.angular.y = 0.0
        self.model_state_req.model_state.twist.angular.z = 0.0
        self.model_state_req.model_state.reference_frame = 'world'

        self.model_config_proxy = rospy.ServiceProxy('/gazebo/set_model_configuration',SetModelConfiguration)
        self.model_config_req = SetModelConfigurationRequest()
        self.model_config_req.model_name = 'hexapod'
        self.model_config_req.urdf_param_name = 'robot_description'
        self.model_config_req.joint_names = self.agent.joint_name_lst
        self.model_config_req.joint_positions = np.zeros(18, dtype=float)

        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.get_model_state_req = GetModelStateRequest()
        self.get_model_state_req.model_name = 'hexapod'
        self.get_model_state_req.relative_entity_name = 'world'

        self.reset_world = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    def _pause(self):
        # pause physics
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_proxy()
        except rospy.ServiceException:
            print('/gazebo/pause_physics service call failed')

        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_world()
        except rospy.ServiceException:
            print('/gazebo/reset_simulation service call failed')

    def _reset(self):
        self._pause()
        # Set model's position from world
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.model_state_proxy(self.model_state_req)
        except rospy.ServiceException:
            print('/gazebo/set_model_state call failed')
        # Set model's joint config
        rospy.wait_for_service('/gazebo/set_model_configuration')
        try:
            self.model_config_proxy(self.model_config_req)
        except rospy.ServiceException:
            print('/gazebo/set_model_configuration call failed')
        # Unpause physics
        try:
            self.unpause_proxy()
        except rospy.ServiceException:
            print('/gazebo/unpause_physics service call failed')
        time.sleep(5)
        # Move Jointsself.agent.
        for i in self.jp.keys():
            self.jp[i] = 0.0
        self.agent.set_angles(self.jp)
        time.sleep(5)

    def _torso(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        pos = self.get_model_state_proxy(self.get_model_state_req)
        return np.array([pos.pose.position.x, pos.pose.position.y])

    def _contact(self, id):
        state = self.agent.force[id].get('contact')
        return state

    def _fn(self, id):
        return self.agent.force[id].get('normal_force')

    def tripod(self, iteration):
        legs = {}
        values = self.agent.get_angles()
        for i in range(6):
            joints = np.array([values.get('tc_' + str(i + 1)), values.get('ctr_' + str(i + 1)), values.get('fti_' + str(i + 1)) - np.pi/2])
            if i % 2 == 0:
                # Dictionary containing all info about the limbs actuation
                data = {'id': i, 'phase': True}
                # Swing phase
                if iteration == 0:
                    p = Kinematics(i).fk(joints)
                else:
                    p = self.walker.p0[i]
                parameters = self.walker.p1_p(p, i)
                legs[i] = {**data, **parameters}
            else:
                # Dictionary containing all info about the limbs actuation
                data = {'id': i, 'phase': False}
                # Stance phase
                if iteration == 0:
                    p = Kinematics(i).fk(joints)
                else:
                    p = self.walker.pos[i]
                parameters = self.walker.p2_p(p, i)
                legs[i] = {**data, **parameters}
        return legs


    def move_joints(self, i):
        p_ = self.tripod(i)
        
        #for _, t in enumerate(self._time):
        ti, t = rospy.get_time(), rospy.get_time()
        while t - ti <= self._time:
            print(t-ti)
            pos = np.zeros(18, dtype=float)
            for id in range(6):
                idx = [id, id + 6, id + 12]
                if p_[id]['phase'] is False:
                    # Stance phase
                    angular_pos = self.walker.p2(p_[id], t-ti, self._contact(id))
                else:
                    # Swing phase
                    angular_pos = self.walker.p1(p_[id], t-ti, self._contact(id))
                #foot = Kinematics(id).fk_tc(angular_pos)
                angles = [np.array([np.rad2deg(angular_pos[0]), np.rad2deg(angular_pos[1]), np.rad2deg(angular_pos[2])])]
                if id == 0:
                    self.l1 = np.concatenate((self.l1, angles), axis=0)
                elif id == 1:
                    self.l2 = np.concatenate((self.l2, angles), axis=0)
                elif id == 2:
                    self.l3 = np.concatenate((self.l3, angles), axis=0)
                elif id == 3:
                    self.l4 = np.concatenate((self.l4, angles), axis=0)
                elif id == 4:
                    self.l5 = np.concatenate((self.l5, angles), axis=0)
                else:
                    self.l6 = np.concatenate((self.l6, angles), axis=0)

                for j, joint in enumerate(idx):
                    if j == 2:
                        pos[joint] = angular_pos[j] + np.pi/2
                    else:
                        pos[joint] = angular_pos[j]

            for k, joint_name in enumerate(self.joints):
                self.jp[joint_name] = pos[k] + self.error[joint_name]
            self.agent.set_angles(self.jp)
            self.agent.rate.sleep()
            actual_pos = self.agent.get_angles()

            for i in self.jp.keys():
                self.error[i] = (actual_pos[i] - self.jp[i]) * self.k
            t = rospy.get_time()

    def step(self, i):
        self.move_joints(i)



if __name__ == '__main__':
    time.sleep(5)

    hexapod = Env()
    try:
        hexapod._reset()
        time.sleep(5)
        for i in range(8):
            hexapod.step(i)


        time.sleep(10)

        hexapod._pause()
        print('Ending simulation...')
        rospy.signal_shutdown('Simulation ended!')
        # Kill the roslaunch process
        # Get a list of all active nodes
        node_list = subprocess.check_output(["rosnode", "list"]).decode().strip().split("\n")

        # Kill each node in the list
        for node in node_list:
            print('Shutting down node: ', node)
            subprocess.call(["rosnode", "kill", node])


    except rospy.ROSInterruptException:
        pass
