import numpy as np

class Kinematics:
    def __init__(self, id):
        self.l1 = 0.063875
        self.l2 = 0.080
        #self.l3 = 0.114 + 0.04
        self.l3 = 0.127998
        
        self._id = id
        
        self.torso_width = 0.2
        self.torso_depth = 0.12
        
        if id == 0 or id == 3:
            self.a = self.torso_depth / 2
        else:
            self.a = np.sqrt(np.power(self.torso_width / 2, 2) + np.power(self.torso_depth / 2, 2))
            
        ro = np.arctan((self.torso_width / 2) / (self.torso_depth /2))
        self.tau = [0, ro, np.pi - ro, np.pi, np.pi + ro, -ro]
        
    def fk(self, arg):
        #print(arg)
        arg['TC'] = np.deg2rad(arg['TC'])
        arg['CTr'] = np.deg2rad(arg['CTr'])
        arg['FTi'] = np.deg2rad(arg['FTi'])
        return np.array([np.cos(self.tau[self._id] + arg['TC']) * (self.l3 * np.cos(arg['CTr'] + arg['FTi']) + self.l2 * np.cos(arg['CTr']) + self.l1)
                         + self.a * np.cos(self.tau[self._id]),
                         np.sin(self.tau[self._id] + arg['TC']) * (self.l3 * np.cos(arg['CTr'] + arg['FTi']) + self.l2 * np.cos(arg['CTr']) + self.l1)
                         + self.a * np.sin(self.tau[self._id]),
                         self.l3 * np.sin(arg['CTr'] + arg['FTi']) + self.l2 * np.sin(arg['CTr'])])
    
    def m_04(self, arg):
        m = np.eye(4, dtype=float)
        m[0, 0] = np.cos(self.tau[self._id] + arg[0]) * np.cos(arg[1] + arg[2])
        m[0, 1] = -np.cos(self.tau[self._id] + arg[0]) * np.sin(arg[1] + arg[2])
        m[0, 2] = np.sin(self.tau[self._id] + arg[0])
        m[0, 3] = np.cos(self.tau[self._id] + arg[0]) * (self.l3 * np.cos(arg[1] + arg[2]) + self.l2 * np.cos(arg[1]) + self.l1) + self.a * np.cos(self.tau[self._id])
        
        m[1, 0] = np.sin(self.tau[self._id] + arg[0]) * np.cos(arg[1] + arg[2])
        m[1, 1] = -np.sin(self.tau[self._id] + arg[0]) * np.sin(arg[1] + arg[2])
        m[1, 2] = -np.cos(self.tau[self._id] + arg[0])
        m[1, 3] = np.sin(self.tau[self._id] + arg[0]) * (self.l3 * np.cos(arg[1] + arg[2]) + self.l2 * np.cos(arg[1]) + self.l1) + self.a * np.sin(self.tau[self._id])
        
        m[2,0] = np.sin(arg[1] + arg[2])
        m[2,1] = np.cos(arg[1] + arg[2])
        m[2,2] = 0
        m[2,3] = self.l3 *np.sin(arg[1] + arg[2]) + self.l2 * np.sin(arg[1])
        
        return m
    
    def transformation_matrix(self, arg):
        m = np.zeros((4,4), dtype=float)
        m[0,0] = np.cos(arg[1])
        m[0,1] = -np.sin(arg[1]) * np.cos(arg[0])
        m[0,2] = np.sin(arg[1]) * np.sin(arg[0])
        m[0,3] = arg[2] * np.cos(arg[1])
        m[1,0] = np.sin(arg[1])
        m[1,1] = np.cos(arg[1]) * np.cos(arg[0])
        m[1,2] = -np.cos(arg[1]) * np.sin(arg[0])
        m[1,3] = arg[2] * np.sin(arg[1])
        m[2,1] = np.sin(arg[0])
        m[2,2] = np.cos(arg[0])
        m[2,3] = arg[3]
        m[3,3] = 1
        return m
    
    def tc(self):
        return self.transformation_matrix([0.0, self.tau[self._id], self.a, 0.0])
    
    def ik (self, foot):
        foot = np.concatenate((np.array(foot), [1]), axis = 0)
        m_torso_tc = self.tc()
        foot_tc = np.linalg.inv(m_torso_tc).dot(foot.T)
        
        theta_1 = np.arctan2(foot_tc[1], foot_tc[0])
        
        ctr = np.array([self.l1 *np.cos(theta_1 + self.tau[self._id]) + self.a *np.cos(self.tau[self._id]), self.l1 * np.sin(theta_1 + self.tau[self._id]) + self.a * np.sin(self.tau[self._id]), 0], dtype=object)
        
        foot_vec = np.delete(foot, [3])
        
        r = np.linalg.norm(foot_vec - ctr)
        beta = np.arccos(np.clip((-np.power(r, 2) + np.power(self.l2, 2) + np.power(self.l3, 2)) / (2 * self.l2 * self.l3), -1.0, 1.0))
        
        theta_3 = -(np.pi - beta)
        
        beta = np.arccos(np.clip((-np.power(self.l3, 2) + np.power(self.l2, 2) + np.power(r, 2)) / (2 * r * self.l2), -1.0, 1.0))
        gamma = np.arcsin(np.clip((ctr[2] - foot_vec[2]) / r, -1.0, 1.0))
        theta_2 = beta - gamma
        
        return np.array([np.rad2deg(theta_1), np.rad2deg(theta_2), np.rad2deg(theta_3)])

        