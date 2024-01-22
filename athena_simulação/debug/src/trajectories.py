#! /usr/bin/env python3

import rospy
import numpy as np
from kinematics import Kinematics
from scipy.interpolate import CubicSpline


class WalkFunction:
    def __init__(self):
        self.height = 0.3
        self.pos = np.zeros((6, 4), dtype=float)
        self._tf = 1.05

    def swing_parameters(self, joints, leg_id, z_error):
        return self.bezier3(joints, leg_id, z_error)

    def bezier3(self, joints, id, z_error):
        # Requires the angular positions
        p0 = Kinematics(id).fk(joints)

        t1, t2, t3 = np.eye(4, dtype=float), np.eye(4, dtype=float), np.eye(4, dtype=float)
        stride = 0.045
        t1[0, 3] = (stride * np.sin(z_error))/4
        t2[0, 3] = (3/4) * stride * np.sin(z_error)
        t3[0, 3] = stride * np.sin(z_error)

        t1[1, 3] = (stride * np.cos(z_error))/4
        t2[1, 3] = (3/4) * stride * np.cos(z_error)
        t3[1, 3] = stride * np.cos(z_error)

        t1[2, 3] = self.height/3
        t2[2, 3] = self.height/3

        # Control points
        p1 = t1.dot(p0.T)
        p2 = t2.dot(p0.T)
        p3 = t3.dot(p0.T)

        parameters = {'p0': p0, 'p1': p1, 'p2': p2, 'p3': p3}
        return parameters

    def p_linear(self, joints, id, z_error):
        # Requires the angular positions
        p0 = Kinematics(id).fk(joints)
        parameters = {'p0': p0, 'yaw': z_error}
        return parameters

    def bezier3_motion(self, params, t):
        id = params.get('id')
        p0 = np.array(params.get('p0'))
        p1 = np.array(params.get('p1'))
        p2 = np.array(params.get('p2'))
        p3 = np.array(params.get('p3'))

        self.pos[id] = np.power(1 - t, 3) * p0 + 3 * t * np.power((1 - t), 2) * p1 + 3 * np.power(t, 2) * (1 - t) * p2 + np.power(t, 3) * p3
        return Kinematics(id).ik(self.pos[id])


    def swing(self, params, t):
        return self.bezier3_motion(params, t)

    def stance(self, params, t):
        id = params.get('id')
        p0 = params.get('p0')
        yaw = params.get('yaw')
        stride = -0.045
        pos = np.array([p0[0] + ((stride*np.sin(yaw))/self._tf) * t, p0[1] + ((stride*np.cos(yaw))/self._tf) * t, p0[2], 1])
        #pos = np.array([p0[0], p0[1] + (stride/self._tf) * t, p0[2], 1])
        return Kinematics(id).ik(pos)
