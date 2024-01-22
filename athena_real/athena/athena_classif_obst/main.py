import os
import csv
import json
import time
import serial
import random
import numpy as np
from ard import Arduino
from conversion import real_to_sim, sim_to_real
#from gpiozero import AngularServo
from kinematics import Kinematics
#from adafruit_servokit import ServoKit
from scipy.interpolate import Akima1DInterpolator, CubicSpline
from struct import *
from pySerialTransfer import pySerialTransfer as txfer

class WalkFunction:
    def __init__(self, time):
        self.h = 0.08
        self.s = 0.045
        #self.tf = time/2.0
        self.tf = 1.0

        #force sensor
        self.tempo = []
        self.t_data = []
        self.t_stance = 0
        self.t_swing = 0

        self.all_trajectory_foot1 = []
        self.all_trajectory_foot2 = []
        self.all_trajectory_foot3 = []
        self.all_trajectory_foot4 = []
        self.all_trajectory_x_foot1 = []
        self.all_trajectory_y_foot1 = []
        self.all_trajectory_x_foot2 = []
        self.all_trajectory_y_foot2 = []
        self.all_trajectory_y_foot4 = []

        self.p_6 = []
        self.p_14 = []
        self.z_t = []
        
    def foot_(self, foot_p):
        f_x, f_y, f_z = [], [], []
        for i in foot_p:
            f_x.extend([i[0]])
            f_y.extend([i[1]])
            f_z.extend([i[2]])
        return f_x, f_y, f_z
        
    def parameters(self, phase, pos, t_matrix, d_h):
        self.functions = []
        for i in range(6):
            if phase[str(i)]['phase'] is False:
                trajectories = self.p1_p(i, pos[str(i)], t_matrix, d_h)
            else:
                trajectories = self.p2_p(i, pos[str(i)], t_matrix, d_h)
            self.functions.append(trajectories)
        
    def p1_p(self, id, pos, t_matrix, d_h):

        # Get the transformation matrix
        r, t, = np.eye(4, dtype=float), np.eye(4, dtype=float)
        for i in range(3):
            for j in range(3):
                r[i, j] = t_matrix[i, j]
        t[1, 3] = self.s
        t[2, 3] = d_h
        transform = np.matmul(r, t)
        transf = np.delete(transform[:,3], [3])
        
        #swing
        p0 = Kinematics(id).fk(pos)

        p1 = np.array([0, transf[1] / 10.5, self.h / 4.6]) + p0
        p2 = np.array([0, 0.224 * transf[1], 0.52 * self.h]) + p0
        p3 = np.array([0, transf[1] / 2, self.h]) + p0
        p4 = np.array([0, (1 - 0.224) * transf[1], 0.52 * self.h]) + p0
        p5 = np.array([0, (9.5 / 10.5) * transf[1], (1 / 4.6) * self.h]) + p0
        p6 = np.array([0, transf[1], 0]) + p0
        
        #stance
        p7 = np.array([transf[0], -transf[1] / 10.5, transf[2] * ((1 + -3 * np.power(1 - 0.225, 2) + 2 * np.power(1 - 0.225, 3)) + 1)]) + p6
        p8 = np.array([transf[0], -0.224 * transf[1], transf[2] * ((1 + -3 * np.power(0.66, 2) + 2 * np.power(0.66, 3)) + 1)]) + p6
        p9 = np.array([transf[0], -(1 / 2) * transf[1], transf[2] * (np.linalg.norm(-3 * np.power(1 / 2, 2) + 2 * np.power(1 / 2, 3)) + 1)]) + p6
        p10 = np.array([transf[0], -(1 - 0.224) * transf[1], transf[2] * ((1 + -3 * np.power(0.34, 2) + 2 * np.power(0.34, 3)) + 1)]) + p6
        p11 = np.array([transf[0], -(9.5 / 10.5) * transf[1], transf[2] * ((1 + -3 * np.power(0.225, 2) + 2 * np.power(0.225, 3)) + 1)]) + p6
        p12 = np.array([transf[0], -transf[1], transf[2]]) + p6
        p13 = np.array([transf[0], -transf[1], transf[2]]) + p6

        p = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13]
    
        
        time = [0, 3 * np.power(0.225, 2) - 2 * np.power(0.225, 3),
                3 * np.power(0.34, 2) - 2 * np.power(0.34, 3),
                3 * np.power(1 / 2, 2) - 2 * np.power(1 / 2, 3),
                3 * np.power(0.66, 2) - 2 * np.power(0.66, 3),
                3 * np.power(1 - 0.225, 2) - 2 * np.power(1 - 0.225, 3), 1,
                (1 + -3 * np.power(1 - 0.225, 2) + 2 * np.power(1 - 0.225, 3)) + 1,
                (1 + -3 * np.power(0.66, 2) + 2 * np.power(0.66, 3)) + 1,
                np.linalg.norm(-3 * np.power(1 / 2, 2) + 2 * np.power(1 / 2, 3)) + 1,
                (1 + -3 * np.power(0.34, 2) + 2 * np.power(0.34, 3)) + 1,
                (1 + -3 * np.power(0.225, 2) + 2 * np.power(0.225, 3)) + 1, 2, 2.05]
        
        trajectories = self.foot_(p)

        # Generate the cubic spline curve
        x = CubicSpline(time, np.array(trajectories[0]))
        y = CubicSpline(time, np.array(trajectories[1]))
        z = CubicSpline(time, np.array(trajectories[2]))
        
        return x, y, z
        
    def p2_p(self, id, pos, t_matrix, d_h):
        # Get the transformation matrix
        r, t, ts = np.eye(4, dtype=float), np.eye(4, dtype=float), np.eye(4, dtype=float)
        for i in range(3):
            for j in range(3):
                r[i, j] = t_matrix[i, j]
        t[1, 3] = self.s
        ts[1, 3] = -self.s
        t[2, 3] = d_h
        ts[2, 3] = d_h
        transform = np.matmul(r, t)
        transf = np.delete(transform[:,3], [3])
        
        #stance
        p0 = Kinematics(id).fk(pos)

        p1 = np.array([transf[0], -transf[1] / 10.5, transf[2] * (3 * np.power(0.225, 2) - 2 * np.power(0.225, 3))]) + p0
        p2 = np.array([transf[0], -0.224 * transf[1], transf[2] * (3 * np.power(0.34, 2) - 2 * np.power(0.34, 3))]) + p0
        p3 = np.array([transf[0], -(1 / 2) * transf[1], transf[2] * (3 * np.power(1 / 2, 2) - 2 * np.power(1 / 2, 3))]) + p0
        p4 = np.array([transf[0], -(1 - 0.224) * transf[1], transf[2] * (3 * np.power(0.66, 2) - 2 * np.power(0.66, 3))]) + p0
        p5 = np.array([transf[0], -(9.5 / 10.5) * transf[1], transf[2] * (3 * np.power(1 - 0.225, 2) - 2 * np.power(1 - 0.225, 3))]) + p0
        p6 = np.array([transf[0], -transf[1], transf[2] * 1]) + p0

        #swing
        p7 = np.array([0, transf[1] / 10.5, self.h / 4.6]) + p6
        p8 = np.array([0, 0.224 * transf[1], 0.52 * self.h]) + p6
        p9 = np.array([0, transf[1] / 2, self.h]) + p6
        p10 = np.array([0, (1 - 0.224) * transf[1], 0.52 * self.h]) + p6
        p11 = np.array([0, (9.5 / 10.5) * transf[1], (1 / 4.6) * self.h]) + p6
        p12 = np.array([0, transf[1], 0]) + p6
        p13 = np.array([0, transf[1], 0]) + p6
        p14 = np.array([0, transf[1], 0]) + p6



        p = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14]

        time = [0, 3 * np.power(0.225, 2) - 2 * np.power(0.225, 3),
                3 * np.power(0.34, 2) - 2 * np.power(0.34, 3),
                3 * np.power(1 / 2, 2) - 2 * np.power(1 / 2, 3),
                3 * np.power(0.66, 2) - 2 * np.power(0.66, 3),
                3 * np.power(1 - 0.225, 2) - 2 * np.power(1 - 0.225, 3), 1,
                (1 + -3 * np.power(1 - 0.225, 2) + 2 * np.power(1 - 0.225, 3)) + 1,
                (1 + -3 * np.power(0.66, 2) + 2 * np.power(0.66, 3)) + 1,
                np.linalg.norm(-3 * np.power(1 / 2, 2) + 2 * np.power(1 / 2, 3)) + 1,
                (1 + -3 * np.power(0.34, 2) + 2 * np.power(0.34, 3)) + 1,
                (1 + -3 * np.power(0.225, 2) + 2 * np.power(0.225, 3)) + 1, 2, 2.03, 2.05]
        
        trajectories = self.foot_(p)

        # Generate the cubic spline curve
        x = CubicSpline(time, np.array(trajectories[0]))
        y = CubicSpline(time, np.array(trajectories[1]))
        z = CubicSpline(time, np.array(trajectories[2]))

        self.p_14.append(p13[2])
        
        self.z_t.append(z(2.0))
        return x, y, z
    
    def trajectory(self, id, t, contact):
        #print("TEMPO________________", t)
        p = [self.functions[id][0](t), self.functions[id][1](t), self.functions[id][2](t)]
        if id == 1:
            self.all_trajectory_foot1.append(self.functions[1][2](t))
            self.all_trajectory_x_foot1.append(self.functions[1][0](t))
            self.all_trajectory_y_foot1.append(self.functions[1][1](t))
        elif id == 2:
            self.all_trajectory_foot2.append(self.functions[2][2](t))
            self.all_trajectory_x_foot2.append(self.functions[2][0](t))
            self.all_trajectory_y_foot2.append(self.functions[2][1](t))
        elif id == 3:
            self.all_trajectory_foot3.append(self.functions[3][2](t))
        elif id == 4:
            self.all_trajectory_foot4.append(self.functions[4][2](t))
            self.all_trajectory_y_foot4.append(self.functions[4][1](t))

        if (id % 2 == 0 and 1.4 < t <= 2.05):
            if (id == 2 and contact is True and self.t_stance != 1):
                self.tempo.append(t)
                self.t_data.append(t)
                self.t_stance = 1
        elif ( 0.4 < t <= 1.0):
            if (id == 1 and contact is True and self.t_swing != 1):
                self.tempo.append(t)
                self.t_data.append(t)
                self.t_swing = 1
        if 1.0 <= t < 1.2 and self.t_swing != 1:
            self.tempo.append(1.0)
            self.t_data.append(1.0)
            self.t_swing = 1
        elif 1.95 <= t <= 2.1 and self.t_stance != 1:
            self.tempo.append(2.0)
            self.t_data.append(2.0)
            self.t_stance = 1
        return Kinematics(id).ik(p)

class Athena:
    def __init__(self, params):
        self.gait = params['gait']
        self.direction = params['dir']
        self.time = params['time']
        
        self.w = 0.056/2
        self.walker = WalkFunction(self.time)
        self.arduino = Arduino()
        self.arduino.readInfo('Done.')
        print('All set, ready to walk.')
        
        self.error = {}
        for i in range(6):
            self.error[str(i)] = self.dictionary2([0, 0, 0])
            
        self.k = 0.598
        self.servo_error = 0.3318
        self.reset()
        
        #sensors
        self.vx = []
        self.ax = []
        self.desvio = []
        self.delta_h = []
        self.foot_c = []
        self.all_forces = []
        self.vx_data = []
        self.ax_data = []
        self.desv_data = []
        self.obstaculo = 0
        self.obst_dinamico = False
        self.detect = 0
        self.dist_obst = []

        self.classif = []
        
    def reset(self):
        pos= {}
        for i in range(6):
            pos[str(i)] = {'TC': 0, 'CTr': 0, 'FTi': -90}
            
        #converter dicionario em lista
        pos_list = self.dict_to_list(sim_to_real(pos))
        self.arduino.write1(pos_list)

        h_relative = []
        for _ in range(5):
            contacts = self.arduino.all_contact()

        joints = real_to_sim(self.dictionary(self.arduino.encoders))
        if len(contacts) != 0:
            for i in range(len(contacts)):
                foot_c = Kinematics(contacts[i]).fk(joints[str(contacts[i])])
                h_relative.append(abs(foot_c[2]))
            self.h_ref = sum(h_relative)/len(h_relative)
        else:
            #self.h_ref = self.arduino.h
            self.h_ref = 0.127998



    def dictionary2(self, arg):
        return {'TC': arg[0],'CTr': arg[1], 'FTi': arg[2]} 
        
    def dictionary(self, arg):
        pos = {}
        j = 0
        for i in range(6):
            pos[str(i)] = {'TC': arg[j],'CTr': arg[j+1], 'FTi': arg[j+2]}
            j += 3
        return pos
    
    def dict_to_list(self, arg):
        pos_list = []
        for i in range(6):
            for j in arg[str(i)].keys():
                pos_list.append(arg[str(i)][j])
        return pos_list
    
    
    def posture(self, iteration):
        # Get all limbs in contact with the ground
        sl = []
        if iteration % 2 == 0: 
            for i in [0, 2, 4]:
                if self.arduino.contact(i) is True:
                    sl.append(i)
        else:
            for i in [1, 3, 5]:
                if self.arduino.contact(i) is True:
                    sl.append(i)
        '''sl = []
        for limb in range(6):
            if self.arduino.contact(limb) is True:
                sl.append(limb)'''
        # Select three legs for the ground plane estimation
        ge = []
        if len(sl) > 3:
            ge = random.sample(sl, 3)
        else:
            ge = sl

        if len(ge) == 3:
            feet_O = {}
            h_relative = []
            joints = real_to_sim(self.dictionary(self.arduino.encoders))
            for id, num in enumerate(ge):
                f = Kinematics(num).fk(joints[str(num)])
                h_relative.append(abs(f[2]))
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


            h_mean = sum(h_relative)/len(h_relative)

            return roll, pitch, h_mean
        else:
            return 0.0, 0.0, 0.127998

        
    def tripod(self, iteration, t_matrix, d_h):
        legs = {}
        for i in range(6):
            if i % 2 == 0:
                legs[str(i)] = {'phase': True}
            else:
                legs[str(i)] = {'phase': False}

        
        self.arduino.write1(self.arduino.encoders)
        angles = real_to_sim(self.dictionary(self.arduino.encoders))
        self.walker.parameters(legs, angles, t_matrix, d_h)
        
    
    def move_joints(self, i):
        #deteção obstaculos
        if 20.0 < self.arduino.dist_ultra < 40.0:
            self.obst_dinamico = True

        if 20.0 < self.arduino.dist_ultra <= 60.0 and self.detect != 1:
            self.obstaculo += 1
            self.detect = 1
        else:
            if self.detect != 1:
                self.obstaculo = 0


        r, p, h = self.posture(i)

        Rx, Ry = np.eye(3, dtype=float), np.eye(3, dtype=float)
        Rx[1, 1] = np.cos(r*self.k_a)
        Rx[1, 2] = -np.sin(r*self.k_a)
        Rx[2, 1] = np.sin(r*self.k_a)
        Rx[2, 2] = np.cos(r*self.k_a)

        Ry[0, 0] = np.cos(p*self.k_b)
        Ry[0, 2] = np.sin(p*self.k_b)
        Ry[2, 0] = -np.sin(p*self.k_b)
        Ry[2, 2] = np.cos(p*self.k_b)

        t_matrix = np.matmul(Rx, Ry)

        d_h = (self.k_h)*(h-self.h_ref)
        #d_h = 0
        self.delta_h.append(d_h)
        self.tripod(i, t_matrix, d_h)
        v_x = []
        a_x = []
        v_x2 = []
        a_x2 = []
        height = []
        height2 = []
        ultrassonico = []
        ultrassonico2 = []

        ti, t = time.time(), time.time()
        while t - ti <= self.time:
            pos = {}
            for id in range(6):                
                pos[str(id)] = self.dictionary2(self.walker.trajectory(id, t-ti, self.arduino.foot_contact(id)))
                # pos[str(id)] = self.dictionary2(self.walker.trajectory(id, t-ti, 0))
                '''pos[str(id)]['TC'] += self.error[str(id)]['TC']
                pos[str(id)]['CTr'] += self.error[str(id)]['CTr']
                pos[str(id)]['FTi'] += self.error[str(id)]['FTi']'''
            angles = sim_to_real(pos)
            pos_list = self.dict_to_list(angles)
            self.arduino.write1(pos_list)
            prev_pos = real_to_sim(angles)
            #self.arduino.write1(self.arduino.encoders)
            actual_pos = real_to_sim(self.dictionary(self.arduino.encoders))
            #actual_pos = self.motors.encoders()
            
            '''for i in range(6):
                for j in pos[str(i)].keys():
                    self.error[str(i)][j] = self.servo_error * self.k'''
                    
            self.all_forces.append(self.arduino.force)
            if t-ti <= 1.0:
                v_x.append(self.arduino.vel_x)
                a_x.append(self.arduino.accel_x)
                height.append(self.arduino.h)
                ultrassonico.append(self.arduino.dist_ultra)
            else:
                v_x2.append(self.arduino.vel_x)
                a_x2.append(self.arduino.accel_x)
                height2.append(self.arduino.h)
                ultrassonico2.append(self.arduino.dist_ultra)
            t = time.time()
        self.vx.append(rms(v_x))
        self.vx_data.append(rms(v_x))
        self.vx.append(rms(v_x2))
        self.vx_data.append(rms(v_x2))
        self.ax.append(rms(a_x))
        self.ax_data.append(rms(a_x))
        self.ax.append(rms(a_x2))
        self.ax_data.append(rms(a_x2))
        h_aux = len(height)
        h2_aux = len(height2)
        self.desvio.append(abs(self.h_ref - height[h_aux-1]))
        self.desv_data.append(abs(self.h_ref - height[h_aux-1]))
        self.desvio.append(abs(self.h_ref - height2[h2_aux-1]))
        self.desv_data.append(abs(self.h_ref - height2[h2_aux-1]))

        d_aux = len(ultrassonico)
        d2_aux = len(ultrassonico2)
        self.dist_obst.append(ultrassonico[d_aux-1])
        self.dist_obst.append(ultrassonico2[d2_aux-1])

        joints = real_to_sim(self.dictionary(self.arduino.encoders))
        for i in range(6):
            self.foot_c.append(Kinematics(i).fk(joints[str(i)]))
        
        
    def step(self, i):
        self.k_a = 0.0
        self.k_b = 0.0
        self.k_h = 0.0
        self.move_joints(i)
        
    def classification(self):
        t = []
        d = []
        terreno = []
        for i in range(len(self.walker.tempo)):
            if self.walker.tempo[i] > 1.0:
                t.append(self.walker.tempo[i] - 1.0)
            else:
                t.append(self.walker.tempo[i])

        for i in range(len(self.desvio)):
            d.append(abs(self.desvio[i]-7.872002))
        
        vel_class = rms(self.vx)
        accel_class = rms(self.ax)
        tempo = rms(t)
        desvio = rms(d)

        if vel_class < 7.0:
            terreno.append(0.0)
        elif vel_class >11.0:
            terreno.append(2.0)
        else:
            terreno.append(1.0)

        if accel_class <= 0.1:
            terreno.append(0.0)
        elif accel_class > 0.145:
            terreno.append(2.0)
        else:
            terreno.append(1.0)

        if tempo > 0.996:
            terreno.append(0.0)
        elif tempo <=0.96:
            terreno.append(2.0)
        else:
            terreno.append(1.0)

        if desvio < 0.7:
            terreno.append(0.0)
        elif desvio >= 1.6:
            terreno.append(2.0)
        else:
            terreno.append(1.0)

        med = np.mean(terreno)

        if med <= 0.5:
            print(med, " : TERRENO REGULAR")
            self.classif.append(0)
        elif med >= 1.5:
            print(med, " : TERRENO IRREGULAR")
            self.classif.append(2)
        else:
            print(med, " : RAMPA")
            self.classif.append(1)
            
        
def rms(d):
    n = len(d)
    square = 0
    for i in range(n):
        square += np.power(d[i], 2)
    return np.sqrt(square/float(n))

if __name__ == '__main__':
    
    path = '/home/isabel/Desktop/athena_ard_new'
    os.chdir(path)
    
    f = open('parameters.json')
    params = json.load(f)
    
    angles={}
    
    hexapod = Athena(params)

    for i in range(8):
        it = i + 1
        hexapod.step(i)
        hexapod.walker.t_stance = 0
        hexapod.walker.t_swing = 0
        if it % 2 == 0:
            hexapod.classification()
            hexapod.walker.tempo = []
            hexapod.vx = []
            hexapod.ax = []
            hexapod.desvio = []

        if hexapod.obst_dinamico == True:
                print(" ------------- OBSTÀCULO PRÓXIMO :", hexapod.arduino.dist_ultra, " cm -------------") 
                hexapod.obst_dinamico = False       

        if hexapod.obstaculo >= 3:
            print("------------- OBSTÁCULO :  ", hexapod.arduino.dist_ultra, " cm -------------")
        hexapod.detect = 0     

        print('FEITO ', i)
        print(' ')

    print(hexapod.walker.t_data)
    print("altura ref", hexapod.h_ref)
    # Write the sensor data to a CSV file
    with open('/home/isabel/Desktop/test_athena_ard/classif/class_obst_test.csv', 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['vel_x', 'accel_x', 'ciclos', 'altura', 'dist'])
        
        print(len(hexapod.vx_data)," ", len(hexapod.ax_data), " ", len(hexapod.walker.t_data), " ", len(hexapod.desv_data))
        for k in range(len(hexapod.vx_data)):
                writer.writerow([hexapod.vx_data[k], hexapod.ax_data[k], hexapod.walker.t_data[k], hexapod.desv_data[k], hexapod.dist_obst[k]])
    #------------------------------------------------

    print("CLASSIFICAÇÃO: ", hexapod.classif)
    # Write the trajectory and force sensor to a CSV file
    if (len(hexapod.walker.all_trajectory_foot1) != len(hexapod.all_forces)):
        for i in range(len(hexapod.all_forces)-len(hexapod.walker.all_trajectory_foot1)):
            hexapod.walker.all_trajectory_foot1.append(0.0)
            hexapod.walker.all_trajectory_foot2.append(0.0)
            hexapod.walker.all_trajectory_foot3.append(0.0)
            hexapod.walker.all_trajectory_foot4.append(0.0)
            hexapod.walker.all_trajectory_x_foot1.append(0.0)
            hexapod.walker.all_trajectory_y_foot1.append(0.0)
            hexapod.walker.all_trajectory_x_foot2.append(0.0)
            hexapod.walker.all_trajectory_y_foot2.append(0.0)
            hexapod.walker.all_trajectory_y_foot4.append(0.0)
    with open('/home/isabel/Desktop/test_athena_ard/classif/forca_class_obst_test.csv', 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['force1', 'force2', 'force3', 'force4', 'force5', 'force6', 'trajectory z 1', 'trajectory z 2', 'trajectory z 3', 
                         'trajectory z 4', 'x f1', 'y f1', 'x f2', 'y f2', 'y f4'])
        for k in range(len(hexapod.all_forces)):
            writer.writerow([hexapod.all_forces[k][0], hexapod.all_forces[k][1], hexapod.all_forces[k][2], 
                             hexapod.all_forces[k][3], hexapod.all_forces[k][4], hexapod.all_forces[k][5], 
                             hexapod.walker.all_trajectory_foot1[k], hexapod.walker.all_trajectory_foot2[k], 
                             hexapod.walker.all_trajectory_foot3[k], hexapod.walker.all_trajectory_foot4[k],
                             hexapod.walker.all_trajectory_x_foot1[k], hexapod.walker.all_trajectory_y_foot1[k], 
                             hexapod.walker.all_trajectory_x_foot2[k], hexapod.walker.all_trajectory_y_foot2[k], hexapod.walker.all_trajectory_y_foot4[k]])
    #------------------------------------------------

    hexapod.arduino.link.close()
