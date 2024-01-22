#! /usr/bin/env python3

import os
import sys
import json
import csv
import time
import rospy
import signal
import random
import subprocess
import numpy as np
import pandas as pd
from ros_client import Athena
from std_srvs.srv import Empty
import matplotlib.pyplot as plt
from kinematics import Kinematics
from gazebo_msgs.msg import ModelState
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest

class WalkFunction:
    def __init__(self):
        self.height = 0.3
        self.pos = np.zeros((6, 4), dtype=float)
        self.p0 = np.zeros((6, 4), dtype=float)
        self._tf = np.round(1.0 , decimals=3) 

        # For time control
        self.tempo = []
        self.flag_t = 0

    def p1_p(self, p0):
        # Requires the angular positions
        #p0 = Kinematics(id).fk(joints)

        s = 0.045

        t1 = np.array([0, s/4, self.height/3, 0])
        t2 = np.array([0, (3/4)*s, self.height/3, 0])
        t3 = np.array([0, s, 0, 0])

        # Control points
        p1 = t1 + p0
        p2 = t2 + p0
        pf = t3 + p0

        parameters = {'p0': p0, 'p1': p1, 'p2': p2, 'p3': pf}
        #parameters = {'p0': p0, 'p1': p1, 'p2': p2, 'p3': pf, 't1': t1, 't2': t2, 't3': t3}
        return parameters

    def p2_p(self, p0, h):
        # Requires the angular positions
        #p0 = Kinematics(id).fk(joints)

        s = 0.045

        parameters = {'p0': p0, 'h': h}
        return parameters

    def p1(self, params, t, contact):
        id = params.get('id')
        p0 = np.array(params.get('p0'))
        p1 = np.array(params.get('p1'))
        p2 = np.array(params.get('p2'))
        p3 = np.array(params.get('p3'))
        t1 = np.array(params.get('t1'))
        t2 = np.array(params.get('t2'))
        t3 = np.array(params.get('t3'))

        if (t >= 1/5 * self._tf and contact is True):
            #print("HERE : ", id, "t:", t)
            if ((id ==1 or id == 2) and self.flag_t != 1 and t != 1.0):
                self.tempo.append(t)
                self.flag_t = 1
            return Kinematics(id).ik(self.pos[id])
        
        else:
            if ((id == 1 or id ==2) and self.flag_t != 1):
                self.tempo.append(t)
            self.pos[id] = np.power(1 - t, 3) * p0 + 3 * t * np.power((1 - t), 2) * p1 + 3 * np.power(t, 2) * (1 - t) * p2 + np.power(t, 3) * p3
            return Kinematics(id).ik(self.pos[id])
        
        

    def p2(self, params, t):
        id = params.get('id')
        p0 = params.get('p0')
        h = params.get('h')

        if t <= self._tf:
            s = -0.045
            self.p0[id] = np.array([p0[0], p0[1] + (s/self._tf) * t, p0[2] + (h/self._tf) * t, 1])
        return Kinematics(id).ik(self.p0[id])


class Env:
    def __init__(self):
        # ROS Client
        self.agent = Athena()
        self.joints = self.agent.joint_name_lst
        self.jp, self.error = {}, {}
        for joint in self.joints:
            self.jp[joint] = 0.0
            self.error[joint] = 0.0

        self._time = np.round(np.arange(0, 1.05, 50e-3), decimals = 3)
        #self._time = np.arange(0, 1.05, 50e-3)
        self.walker = WalkFunction()

        self._w = 0.056/2

        # For the posture evaluation
        self.roll = []
        self.pitch = []

        # For plot position
        self._posicaoZ = []
        self.altura = []
        #For CLASSIFICATION
        self.vel_x = []
        self.plot_vel = []
        self.desvio = []
        self.oscilacoes = []
        self.regularidade = []
        self.rms_imu = []
        self.accel_x = []
        #self.desvio_total = []
        self.terreno = []
        self.obstaculo =[]

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
        

        self.reset_world = rospy.ServiceProxy('/gazebo/reset_simul            t = t + 0.05ation', Empty)

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
        # Move Joints
        for i in self.jp.keys():
            self.jp[i] = 0.0
        self.agent.set_angles(self.jp)
        time.sleep(8)
        for _ in range(5):
            self.h_ref = self.agent.depth + self._w
        time.sleep(0.5)
        '''for x in range(8):
            print(self.agent.sonar[x])'''

    def _torso(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        pos = self.get_model_state_proxy(self.get_model_state_req)
        return np.array([pos.pose.position.x, pos.pose.position.y])

    def _contact(self, id):
        state = self.agent.force[id].get('contact')
        return state

    def _fn(self, id):
        return self.agent.force[id].get('normal_force')
    
    def _posture(self):
        # Get all limbs in contact with the ground
        sl = []
        for limb in range(6):
            if self._contact(limb) is True:
                sl.append(limb)
        # Select three legs for the ground plane estimation
        ge = []
        if len(sl) > 3:
            ge = random.sample(sl, 3)
        else:
            ge = sl
        # Get the feet relative coordinates
        if len(ge) == 3:
            feet_O = {}
            values = self.agent.get_angles()
            for id, num in enumerate(ge):
                joints = np.array([values.get('tc_' + str(num + 1)), values.get('ctr_' + str(num + 1)), values.get('fti_' + str(num + 1)) - np.pi/2])
                f_vec = Kinematics(num).fk(joints)
                f = np.delete(f_vec, [3])
                feet_O[id] = {'foot': f}
            # Estimate the ground's norm vector
            v1 = feet_O[2]['foot'] - feet_O[0]['foot']
            v2 = feet_O[1]['foot'] - feet_O[0]['foot']
            u_ground = np.cross(v1, v2)
            # Set the unitary vector eta from the local coordinates of the torso
            eta = np.array([0, 1])
            # Rotation along the X-axis
            u_yz = np.delete(u_ground, [0])
            roll = 0.0 if np.linalg.norm(u_yz) == 0.0 else np.arccos((np.dot(u_yz, eta)) / (np.linalg.norm(u_yz) * np.linalg.norm(eta)))
            if roll >= np.pi/2:
                roll = np.pi - roll
            if u_yz[0] < 0:
                roll *= -1
            # Rotation along the Y-axis
            u_xz = np.delete(u_ground, [1])
            pitch = 0.0 if np.linalg.norm(u_xz) == 0.0 else np.arccos((np.dot(u_xz, eta)) / (np.linalg.norm(u_xz) * np.linalg.norm(eta)))
            if pitch >= np.pi/2:
                pitch = np.pi - pitch
            if u_xz[0] < 0:
                pitch *= -1
            self.roll.append(roll)
            self.pitch.append(pitch)
            # Height adjustment
            for _ in range(5):
                h = self.agent.depth + self._w#* np.cos(roll)
            return roll, pitch, h#roll, pitch
        else:
            return 0.0, 0.0, self.h_ref

    def tripod(self, iteration, d_h):
        legs = {}
        values = self.agent.get_angles()
        for i in range(6):
            joints = np.array([values.get('tc_' + str(i + 1)), values.get('ctr_' + str(i + 1)), values.get('fti_' + str(i + 1)) - np.pi/2])
            if iteration == 0:
                p = Kinematics(i).fk(joints)
            else:
                p = self.walker.p0[i]
            if iteration % 2 == 0:
                if i % 2 == 0:
                    # Dictionary containing all info about the limbs actuation
                    data = {'id': i, 'phase': True}
                    if iteration == 0:
                        p = Kinematics(i).fk(joints)
                    else:
                        p = self.walker.p0[i]
                    # Swing phase       
                    parameters = self.walker.p1_p(p)
                    legs[i] = {**data, **parameters}
                else:
                    # Dictionary containing all info about the limbs actuation
                    data = {'id': i, 'phase': False}
                    if iteration == 0:
                        p = Kinematics(i).fk(joints)
                    else:
                        p = self.walker.pos[i]     
                    # Stance phase
                    parameters = self.walker.p2_p(p, d_h)
                    legs[i] = {**data, **parameters}

            else:
                if i % 2 == 0:
                    # Dictionary containing all info about the limbs actuation
                    data = {'id': i, 'phase': False}
                    if iteration == 0:
                        p = Kinematics(i).fk(joints)
                    else:
                        p = self.walker.pos[i]     
                    # Swing phase                  
                    parameters = self.walker.p2_p(p, d_h)
                    legs[i] = {**data, **parameters}
                else:
                    # Dictionary containing all info about the limbs actuation
                    data = {'id': i, 'phase': True}
                    if iteration == 0:
                        p = Kinematics(i).fk(joints)
                    else:
                        p = self.walker.p0[i]     
                    # Stance phase
                    parameters = self.walker.p1_p(p)
                    legs[i] = {**data, **parameters}

        return legs
   
    


    def move_joints(self, i):
        r, p, h = self._posture()
        d_h = (self.k_h)*(h-self.h_ref)
        #d_h = h - self.h_ref
        #d_h = 0
        it = i
        p_ = self.tripod(i, d_h)
        vx = []
        ax = []
        for _, t in enumerate(self._time):
            pos = np.zeros(18, dtype=float)
            for id in range(6):
                idx = [id, id + 6, id + 12]
                if p_[id]['phase'] is False:
                    # Stance phase
                    angular_pos = self.walker.p2(p_[id], t)
                else:
                    # Swing phase
                    angular_pos = self.walker.p1(p_[id], t, self._contact(id))
                    #foot = Kinematics(id).fk_tc(angular_pos)

                for j, joint in enumerate(idx):
                    if j == 2:                        
                        pos[joint] = angular_pos[j] + np.pi/2
                    else:
                        #print(angular_pos[j])
                        pos[joint] = angular_pos[j]

            for k, joint_name in enumerate(self.joints):
                self.jp[joint_name] = pos[k] + self.error[joint_name]
            self.agent.set_angles(self.jp)
            self.agent.rate.sleep()
            actual_pos = self.agent.get_angles()

            for i in self.jp.keys():
                self.error[i] = (actual_pos[i] - self.jp[i]) * self.k
            vx.append(self.agent.ang_v[0])
            ax.append(self.agent.accel[0])
        # posicao = Kinematics(2).fk(angular_pos)
        # self._posicao.append(posicao[2])
        #self.get_model_state_res = GetModelStateResponse()
        #print("posicao z", self.get_model_state_res.pose.position.z)
        posZ = self.agent._posicao.z
        self._posicaoZ.append(posZ)
        self.altura.append(h)
        self.desvio.append(abs(self.h_ref - h))
        self.vel_x.append(rms(vx))
        self.plot_vel.append(rms(vx))
        self.accel_x.append(rms(ax))
        '''for x in range(8):
            if 0.2 < self.agent.sonar[x] <= 1.0:
                self.obstaculo.append([self.agent.sonar[x], x, it])
                print("Obstáculo!", self.obstaculo)
            elif self.agent.sonar[x] < 0.2:
                self.obstaculo.append([1.0, x, it])
                print("Obstáculo!", self.obstaculo)'''

    def step(self, i, ka, kb, kh):
        self.k_a = ka
        self.k_b = kb
        self.k_h = kh
        for x in range(8):
            if 0.2 < self.agent.sonar[x] <= 1.0:
                self.obstaculo.append([self.agent.sonar[x], x, i])
                print("Obstáculo!", self.obstaculo)
            elif self.agent.sonar[x] < 0.2:
                self.obstaculo.append([0.0, x, i])
                print("Obstáculo!", self.obstaculo)
        self.move_joints(i)

    def classification(self, dt, imu_vx, desvio_total, it):
        it = it + 1
        if (it % 2 == 0 and it != 0):

            i = 0
            inc = 0
            print("osc", dt)
            for i in range(len(dt)):
                if dt[i] < 1.0:
                    inc += 1                        
            if inc >= 1:
                self.oscilacoes.append(inc)
                self.regularidade.append(0)
                #print("Irregularidades")
            else:
                self.oscilacoes.append(inc)
                self.regularidade.append(1)
                #print("REGULAR (t)")

            self.rms_imu.append(rms(imu_vx))

            if( it % 4 == 0 and it != 0):
                incremento = 0
                if(self.regularidade[0] == 1 and self.regularidade[1] == 1):
                    valor_rms = 0
                    #self.terreno.append(0)
                    #print("TERRENO REGULAR")
                elif(self.regularidade[0] == 0 and self.regularidade[1] == 0):                
                    if ((self.rms_imu[0] + self.rms_imu[1])/2) < 0.20:
                        valor_rms = 1
                        #print("RAMPA")
                    elif ((self.rms_imu[0] + self.rms_imu[1])/2) >= 0.20:
                        valor_rms = 2
                        #print("TESTBED")

                elif((self.regularidade[0] == 0 and self.regularidade[1] == 1) or (self.regularidade[0] == 1 and self.regularidade[1] == 0)):
                    if ((self.rms_imu[0] + self.rms_imu[1])/2) < 0.14:
                        for j in range(len(self.oscilacoes)):
                            if self.oscilacoes[j] < 1.0:
                                incremento += 1
                        if(incremento <= 1):
                            valor_rms = 0
                            #print("REGULAR")
                        else:
                            valor_rms = 1
                            #print("RAMPA")
                    elif 0.14 <= ((self.rms_imu[0] + self.rms_imu[1])/2) < 0.20:
                        valor_rms = 1
                        #print("RAMPA")
                    elif ((self.rms_imu[0] + self.rms_imu[1])/2) >= 0.20:
                        valor_rms = 2
                        #print("TESTBED")


                maximo = max(desvio_total)
                if (maximo < 0.009):
                    valor_desvio = 0
                elif(0.009 <= maximo < 0.018):
                    valor_desvio = 1
                else:
                    valor_desvio = 2

                if(valor_rms == 0 and valor_desvio == 0):
                    self.terreno.append(0)
                    print("TERRENO: REGULAR")
                elif(valor_rms == 1 and valor_desvio == 1):
                    self.terreno.append(1)
                    print("TERRENO: RAMPA")
                elif(valor_rms == 2 and valor_desvio == 2):
                    self.terreno.append(2)
                    print("TERRENO IRREGULAR")
                elif(valor_rms == 0 and valor_desvio != 0):
                    if len(self.terreno) == 0:
                        self.terreno.append(0)
                    elif self.terreno[-1] == 0:
                        self.terreno.append(0)
                        print("TERRENO REGULAR")
                    elif self.terreno[-1] == 1 and valor_desvio == 1:
                        self.terreno.append(1)
                        print("TERRENO: RAMPA")
                    elif self.terreno[-1] == 2 and valor_desvio == 2:
                        self.terreno.append(2)
                        print("TERRENO IRREGULAR")
                    else:
                        self.terreno.append(0)
                        print("TERRENO REGULAR")                        
                elif(valor_rms != 0 and valor_desvio == 0):
                    if len(self.terreno) == 0:
                        self.terreno.append(0)
                    elif self.terreno[-1] == 0:
                        self.terreno.append(0)
                        print("TERRENO REGULAR")
                    elif self.terreno[-1] == 1 and valor_rms == 1:
                        self.terreno.append(1)
                        print("TERRENO: RAMPA")
                    elif self.terreno[-1] == 2 and valor_rms == 2:
                        self.terreno.append(2)
                        print("TERRENO IRREGULAR")
                    else:
                        self.terreno.append(valor_rms)
                        print("TERRENO: ", valor_rms)
                elif(valor_rms == 1 and valor_desvio != 1):
                    if len(self.terreno) == 0:
                        self.terreno.append(1)
                    if self.terreno[-1] == 1:
                        self.terreno.append(1)
                        print("TERRENO: RAMPA")
                    elif self.terreno[-1] == 0 and valor_desvio == 0:
                        self.terreno.append(0)
                        print("TERRENO: REGULAR")
                    elif self.terreno[-1] == 2 and valor_desvio == 2:
                        self.terreno.append(2)
                        print("TERRENO IRREGULAR")
                    else:
                        self.terreno.append(1)
                        print("TERRENO: RAMPA") 
                elif(valor_rms != 1 and valor_desvio == 1):
                    if len(self.terreno) == 0:
                        self.terreno.append(1)
                    elif self.terreno[-1] == 1:
                        self.terreno.append(1)
                        print("TERRENO: RAMPA")
                    elif self.terreno[-1] == 0 and valor_rms == 0:
                        self.terreno.append(0)
                        print("TERRENO: REGULAR")
                    elif self.terreno[-1] == 2 and valor_rms == 2:
                        self.terreno.append(2)
                        print("TERRENO IRREGULAR")
                    else:
                        self.terreno.append(valor_rms)
                        print("TERRENO: ", valor_rms)
                elif(valor_rms == 2 and valor_desvio != 2):
                    if len(self.terreno) == 0:
                        self.terreno.append(2)
                    elif self.terreno[-1] == 2:
                        self.terreno.append(2)
                        print("TERRENO IRREGULAR")
                    elif self.terreno[-1] == 0 and valor_desvio == 0:
                        self.terreno.append(0)
                        print("TERRENO: REGULAR")
                    elif self.terreno[-1] == 1 and valor_desvio == 1:
                        self.terreno.append(1)
                        print("TERRENO: RAMPA")
                    else:
                        self.terreno.append(2) 
                        print("TERRENO IRREGULAR")
                elif(valor_rms != 2 and valor_desvio == 2):
                    if len(self.terreno) == 0:
                        self.terreno.append(2)
                    elif self.terreno[-1] == 2:
                        self.terreno.append(2)
                        print("TERRENO IRREGULAR")
                    elif self.terreno[-1] == 0 and valor_rms == 0:
                        self.terreno.append(0)
                        print("TERRENO: REGULAR")
                    elif self.terreno[-1] == 1 and valor_rms == 1:
                        self.terreno.append(1)
                        print("TERRENO: RAMPA")
                    else:
                        self.terreno.append(valor_rms)
                        print("TERRENO: ", valor_rms)
                return 2
            return 1


def rms(d):
    n = len(d)
    square = 0
    for i in range(n):
        square += np.power(d[i], 2)
    return np.sqrt(square/float(n))

if __name__ == '__main__':
    time.sleep(5)

    ka = 0.0
    kb = 0.0
    kh = 0.6

    tempo_ciclo = []
    vel_x_temp = []
    plot_tempo = []
    hexapod = Env()
    try:
        hexapod._reset()
        time.sleep(5)
        print("h_ref", hexapod.h_ref)
        for i in range(40):
            hexapod.walker.tempo = []
            hexapod.step(i, ka, kb, kh)
            if hexapod.walker.tempo != []:
                tempo_ciclo.append(max(hexapod.walker.tempo))
                plot_tempo.append(max(hexapod.walker.tempo))
            hexapod.walker.flag_t = 0
            classif = hexapod.classification(tempo_ciclo, hexapod.vel_x, hexapod.desvio, i)
            if classif == 1:
                hexapod.vel_x = []
                tempo_ciclo = []
                #hexapod.desvio = []
            if classif == 2:
                hexapod.oscilacoes = []
                hexapod.regularidade = []
                hexapod.rms_imu = []
                hexapod.desvio = []

        print("CLASSIFICAÇÂO: ", hexapod.terreno)
        # #POSIÇAO EM Z DO ROBO
        # plt.plot(hexapod._posicaoZ, label='posição em z')
        # plt.legend()
        # plt.show()
        # #TEMPO DE CICLOS DA SWING
        # #plt.plot(hexapod.walker.tempo, label='ciclos de swing (perna1)')
        # plt.plot(plot_tempo, label='ciclos de swing (perna 1 e 2)')
        # plt.legend()
        # plt.show()
        # #IMU - VELOCIDADE ANGULAR EM X
        # plt.plot(hexapod.plot_vel, label = 'velocidade angular em x')
        # plt.legend()
        # plt.show()
        
        # # Write the data to a CSV file
        # with open('/home/isabel/Desktop/tese/obst3.csv', 'w') as csvfile:
        #     writer = csv.writer(csvfile)
        #     writer.writerow(['vel_x', 'accel_x', 'ciclos', 'desvio'])
        #     for k in range(len(hexapod.plot_vel)):
        #             writer.writerow([hexapod.plot_vel[k], hexapod.accel_x[k], plot_tempo[k], abs(hexapod.h_ref-hexapod.altura[k])])

        # Write the data to a CSV file
        with open('/home/isabel/Desktop/tese/dist_rampa.csv', 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['s0', 's1', 's2', 's3', 's4', 's5', 's6', 's7'])
            for k in range(int(len(hexapod.obstaculo)/8)):
                    aux = []
                    k1 = k * 8
                    for k2 in range(8):
                        aux.append(hexapod.obstaculo[k1+k2][0])
                    writer.writerow(aux)

        #rospy.spin()


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
