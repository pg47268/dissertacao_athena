import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from kinematics import Kinematics
from scipy.interpolate import CubicSpline, Akima1DInterpolator


def foot_(foot_p):
    f_x, f_y, f_z = [], [], []
    for i in foot_p:
        f_x.extend([i[0]])
        f_y.extend([i[1]])
        f_z.extend([i[2]])
    return f_x, f_y, f_z


def spline(t, id):
    s = 0.045
    h = 0.025

    p0 = Kinematics(id).fk([0, 0, 90])
    p0_vec = np.delete(p0, [3])

    p1 = np.array([0, s / 10.5, h / 4.6]) + p0_vec

    p3 = np.array([0, 0.224 * s, 0.52 * h]) + p0_vec
    p4 = np.array([0, s / 2, h]) + p0_vec
    p5 = np.array([0, (1-0.224) * s, 0.52 * h]) + p0_vec
    p7 = np.array([0, (9.5 / 10.5) * s, (1 / 4.6) * h]) + p0_vec
    p8 = np.array([0, s, 0]) + p0_vec

    p9 = np.array([0, -s / 10.5, 0]) + p8
    p10 = np.array([0, -0.224 * s, 0]) + p8
    p12 = np.array([0, -(1 / 2) * s, 0]) + p8
    p14 = np.array([0, -(1-0.224)* s, 0]) + p8
    p15 = np.array([0, -(9.5 / 10.5) * s, 0]) + p8
    p16 = np.array([0, -s, 0]) + p8

    p = [p0, p1, p3, p4, p5, p7, p8, p9, p10, p12, p14, p15, p16]

    time = [0, 3 * np.power(0.225, 2) - 2 * np.power(0.225, 3),
            3 * np.power(0.34, 2) - 2 * np.power(0.34, 3),
            3 * np.power(1 / 2, 2) - 2 * np.power(1 / 2, 3),
            3 * np.power(0.66, 2) - 2 * np.power(0.66, 3),
            3 * np.power(1 - 0.225, 2) - 2 * np.power(1 - 0.225, 3), 1,
            (1 + -3 * np.power(1 - 0.225, 2) + 2 * np.power(1 - 0.225, 3)) + 1,
            (1 + -3 * np.power(0.66, 2) + 2 * np.power(0.66, 3)) + 1,
            np.linalg.norm(-3 * np.power(1 / 2, 2) + 2 * np.power(1 / 2, 3)) + 1,
            (1 + -3 * np.power(0.34, 2) + 2 * np.power(0.34, 3)) + 1,
            (1 + -3 * np.power(0.225, 2) + 2 * np.power(0.225, 3)) + 1, 2]
    print(time)


    trajectories = foot_(p)

    # Generate the cubic spline curve
    x = CubicSpline(time, np.array(trajectories[0]))
    y = CubicSpline(time, np.array(trajectories[1]))
    z = CubicSpline(time, np.array(trajectories[2]))

    inter_xi = x.derivative(nu=1)
    inter_xii = x.derivative(nu=2)

    inter_yi = y.derivative(nu=1)
    inter_yii = y.derivative(nu=2)

    inter_zi = z.derivative(nu=1)
    inter_zii = z.derivative(nu=2)

    #return x, y, z, inter_xi, inter_yi, inter_zi, inter_xii, inter_yii, inter_zii
    return x(t), y(t), z(t)


def spline_inv(t, id):
    s = 0.045
    h = 0.03

    time = [0, 0.3, 0.425, 0.5, 0.575, 0.7, 1, 1.2, 1.35, 1.45, 1.5, 1.55, 1.65, 1.8, 2]
    # stance
    p0 = Kinematics(id).fk([0, 0, 90])
    p0_vec = np.delete(p0, [3])
    p1 = np.array([0, -s / 45, 0]) + p0_vec
    p2 = np.array([0, -1 / 4 * s, 0]) + p0_vec
    p3 = np.array([0, -s / 2, 0]) + p0_vec
    p4 = np.array([0, -3 / 4 * s, 0]) + p0_vec
    p5 = np.array([0, -s + s / 45, 0]) + p0_vec
    p6 = np.array([0, -s, 0]) + p0_vec
    # swing
    p7 = np.array([0, -s / 45, h / 100]) + p6
    p8 = np.array([0, -s / 11.25, 4 / 10 * h]) + p6
    p9 = np.array([0, s / 5, 9 / 10 * h]) + p6
    p10 = np.array([0, s / 2, h]) + p6
    p11 = np.array([0, 4 / 5 * s, 9 / 10 * h]) + p6
    p12 = np.array([0, s + s / 11.25, 4 / 10 * h]) + p6
    p13 = np.array([0, s + s / 45, h / 100]) + p6
    p14 = np.array([0, s, 0]) + p6

    p = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14]

    trajectories = foot_(p)

    # Generate the cubic spline curve
    x = CubicSpline(time, np.array(trajectories[0]))
    y = CubicSpline(time, np.array(trajectories[1]))
    z = CubicSpline(time, np.array(trajectories[2]))

    inter_xi = x.derivative(nu=1)
    inter_xii = x.derivative(nu=2)

    inter_yi = y.derivative(nu=1)
    inter_yii = y.derivative(nu=2)

    inter_zi = z.derivative(nu=1)
    inter_zii = z.derivative(nu=2)

    #return x, y, z, inter_xi, inter_yi, inter_zi, inter_xii, inter_yii, inter_zii
    return x(t), y(t), z(t)


def spline_t(t, id):
    s = 0.045
    h = 0.03


    p0 = Kinematics(id).fk([0, 0, 90])
    p0_vec = np.delete(p0, [3])

    p1 = np.array([0, -s / 10.5, h / 13]) + p0_vec
    p2 = np.array([0, -s / 7.5, h / 7.6]) + p0_vec
    p4 = np.array([0, -s / 5.5, (2 / 5) * h]) + p0_vec
    p5 = np.array([0, s / 3.8, 9.5/10 * h]) + p0_vec
    p6 = np.array([0, s / 2, h]) + p0_vec
    p7 = np.array([0, (4 / 5) * s, 9.5 / 10 * h]) + p0_vec
    p8 = np.array([0, s + (s / 5.75), (4 / 5) * h]) + p0_vec
    p9 = np.array([0, s + s / 7.5, h / 5]) + p0_vec
    p10 = np.array([0, s + s / 8, h / 8]) + p0_vec
    p11 = np.array([0, s + s / 11.25, h / 12]) + p0_vec
    p12 = np.array([0, s, 0]) + p0_vec

    p13 = np.array([0, -s / 11, -0.00005]) + p12
    p14 = np.array([0, -s / 7, -0.0002]) + p12
    p15 = np.array([0, -s / 2, -0.0005]) + p12
    p16 = np.array([0, -(6 / 7) * s, -0.0002]) + p12
    p17 = np.array([0, -(10 / 11) * s, -0.00005]) + p12
    p18 = np.array([0, -s, 0]) + p12

    p = [p0, p1, p2, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p18]
    # p = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p12, p13, p14]
    # time = [0, 0.15, 0.3, 0.4, 0.5, 0.6, 0.7, 0.85, 1, 1.2, 1.45, 1.65, 1.8, 2]
    # time = [0, 0.4, 0.7, 0.9, 1, 1.1, 1.3, 1.6, 2, 2.6, 2.85, 3, 3.15, 3.4, 4]
    # time = [0, 0.2, 0.35, 0.45, 0.5, 0.55, 0.65, 0.8, 1, 1.3, 1.425, 1.5, 1.575, 1.7, 2]
    time = [0, 0.0825, 0.164, 0.327, 0.451, 0.5, 0.59, 0.672, 0.754, 0.836, 0.918, 1.0, 1.12, 1.17, 1.5, 1.83, 1.88, 2]


    trajectories = foot_(p)

    # Generate the cubic spline curve
    x = Akima1DInterpolator(time, np.array(trajectories[0]))
    y = Akima1DInterpolator(time, np.array(trajectories[1]))
    z = Akima1DInterpolator(time, np.array(trajectories[2]))

    return x(t), y(t), z(t)

def spline_inv_t(t, id):
    s = 0.045
    h = 0.025

    p0 = Kinematics(id).fk([0, 0, 90])
    p0_vec = np.delete(p0, [3])

    p13 = np.array([0, -s / 12, -0.0005]) + p0_vec
    p14 = np.array([0, -s / 7, -0.00055]) + p0_vec
    p16 = np.array([0, -(6 / 7) * s, -0.00055]) + p0_vec
    p17 = np.array([0, -(11 / 12) * s, -0.0005]) + p0_vec
    p18 = np.array([0, -s, 0]) + p0_vec

    p4 = np.array([0, s / 8, 0.45 * h]) + p18
    p6 = np.array([0, s / 2, h]) + p18
    p8 = np.array([0, s * (7 / 8), 0.45 * h]) + p18
    p12 = np.array([0, s, 0]) + p18

    p = [p0, p13, p14, p16, p17, p18, p4, p6, p8, p12]
    time = [0, 0.125, 0.21, 0.79, 0.885, 1.1, 1.355, 1.65, 1.855, 2.2]

    trajectories = foot_(p)

    # Generate the cubic spline curve
    x = Akima1DInterpolator(time, np.array(trajectories[0]))
    y = Akima1DInterpolator(time, np.array(trajectories[1]))
    z = Akima1DInterpolator(time, np.array(trajectories[2]))

    return x(t), y(t), z(t)


if __name__ == '__main__':

    fig = plt.figure()
    tempo = np.arange(0, 2.0, 5e-3)
    i = 0

    f = spline(tempo, 0)
    f_inv = spline_inv(tempo, 0)

    #plt.plot(tempo, f[0](tempo), label='x')
    #plt.plot(tempo, f[1](tempo), label='y')
    #plt.plot(tempo, f[2](tempo), label='z')

    #plt.plot(tempo, f[3](tempo), label='vel x')
    '''plt.plot(tempo, f[4](tempo), label='vel y')
    plt.plot(tempo, f[5](tempo), label='vel z')'''
    '''plt.plot(tempo, f_inv[3](tempo), label='vel inv x')
    plt.plot(tempo, f_inv[4](tempo), label='vel inv y')
    plt.plot(tempo, f_inv[5](tempo), label='vel inv z')'''
    #plt.legend()
    #plt.show()

    #plt.plot(tempo, f[6](tempo), label='acc x')
    '''plt.plot(tempo, f[7](tempo), label='acc y')
    plt.plot(tempo, f[8](tempo), label='acc z')'''
    '''plt.plot(tempo, f_inv[6](tempo), label='acc inv x')
    plt.plot(tempo, f_inv[7](tempo), label='acc inv y')
    plt.plot(tempo, f_inv[8](tempo), label='acc inv z')'''
    '''plt.legend()
    plt.show()'''

    #print("Y-direction: ", f[7](0.0), f[7](2.0), "Z-direction: ", f[8](0.0), f[8](2.0))

    ax = fig.add_subplot(projection='3d')
    for _, t in enumerate(tempo):
        #x = spline(t, i)
        s = spline_inv(t, i)
        ax.scatter(s[0], s[1], s[2])
        #ax.scatter(x[0], x[1], x[2])
    plt.legend()
    plt.show()
