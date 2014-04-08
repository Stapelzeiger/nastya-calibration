import csv
import numpy as np
from math import *
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab

import robot_base_mixer

acc_x_global = [0, 0]

class EncData:
    def __init__(self, csv_row):
        self.timestamp = int(csv_row[0])
        self.enc0 = int(csv_row[1])
        self.enc1 = int(csv_row[2])
        self.enc2 = int(csv_row[3])
    def __repr__(self):
        return 'enc@{}'.format(self.timestamp)


def int32(val):
    if val > 2**31-1:
        val -= 2**32
    if val < -2**31:
        val += 2**32
    return val

class EncSpeed:
    def __init__(self, a, b):
        ENCODER_RESOLUTION = 16384
        self.timestamp = b.timestamp
        self.delta_t = int32(b.timestamp - a.timestamp) * 1.0 / 1000000
        self.w0 = int32(b.enc0 - a.enc0)/self.delta_t / ENCODER_RESOLUTION * 2 * pi
        self.w1 = int32(b.enc1 - a.enc1)/self.delta_t / ENCODER_RESOLUTION * 2 * pi
        self.w2 = int32(b.enc2 - a.enc2)/self.delta_t / ENCODER_RESOLUTION * 2 * pi

        Q_vector = np.array([[(0.00005)**2],
                             [(0.00005)**2],
                             [(0.00005)**2]])

        self.Q = np.identity(3) * Q_vector

    def __repr__(self):
        return 'encspeed@{}'.format(self.timestamp)

    def get_vector(self):
        return np.array([[self.w0], [self.w1], [self.w2]])

    def kalman(self, kalman_filter):
        kalman_filter.measurement_update(self.get_vector(), Q = self.Q)

class IMUData:
    def __init__(self, csv_row):
        self.timestamp = int(csv_row[0])
        self.acc_x = float(csv_row[1])
        self.acc_y = float(csv_row[2])
        self.acc_z = float(csv_row[3])
        self.gyro_x = float(csv_row[4])
        self.gyro_y = float(csv_row[5])
        self.gyro_z = float(csv_row[6])

# this is the error of the state-transition + control
        R_vector = np.array([[0.0000000000001],   # pos_x
                             [0.0000000000001],   # pos_y
                             [0.0000000000001],   # theta
                             [0.08**2],   # vel_x
                             [0.08**2],   # vel_y
                             [0.0000003**2],   # omega
                             [0.000000000001],   # gyro_z_null
                             [0.000000000001],   # acc_x_null
                             [0.000000000001],   # acc_y_null
                             [0.0000000000000001],   # imu_orientation
                             [0.0000000000000001],   # D0
                             [0.0000000000000001],   # D1
                             [0.0000000000000001],   # D2
                             [0.0000000000000001],   # R0
                             [0.0000000000000001],   # R1
                             [0.0000000000000001]])  # R2

        self.R = np.identity(16) * R_vector

    def __repr__(self):
        return 'imu@{}'.format(self.timestamp)

    def kalman(self, kalman_filter):
        kalman_filter.control_update(self, self.R)
        # kalman_filter.measurement_update(np.array([[self.gyro_z]]),
        #                                  lambda s: np.array([[s[5][0]]]),
        #                                  np.array([[0.5**2]]))
        state_transition_fn.lastcall = self.timestamp


def read_csv(csv_file, constr):
    print(csv_file)
    data = []
    for line, row in enumerate(csv.reader(open(csv_file), delimiter=',')):
        try:
            data.append(constr(row))
        except Exception as e:
            print('invalid datapoint line: ' + str(line+1) + ' (' + str(e) + ')')
    return data

enc_data = sorted(read_csv('data/enc_01.csv', EncData), key=lambda x: x.timestamp)
imu_data = sorted(read_csv('data/imu_01.csv', IMUData), key=lambda x: x.timestamp)

enc_speed_data = []
for i in range(len(enc_data) - 1):
    enc_speed_data.append(EncSpeed(enc_data[i], enc_data[i+1]))



data = sorted((enc_speed_data + imu_data), key=lambda x: x.timestamp)



def simple_pos_integrate_from_enc_speed(enc_speed_data):
    px = 0
    py = 0
    theta = 0
    for enc in enc_speed_data:
        vx, vy, omega = robot_base_mixer.wheel_to_robot([enc.w0, enc.w1, enc.w2])
        px += (vx * cos(theta) - vy * sin(theta)) * enc.delta_t
        py += (vx * sin(theta) + vy * cos(theta)) * enc.delta_t
        theta += omega * enc.delta_t

    print('enc integration px py theta')
    print(px)
    print(py)
    print(theta)
    return (px, py, theta)

def simple_pos_integrate_from_imu(imu_data):
    zax = 0
    zay = 0
    zgz = 0
    cnt = 0
    first_timestamp = imu_data[0].timestamp
    for imu in imu_data: # find imu zero during fist 2.5 seconds
        if 1.0*int32(imu.timestamp - first_timestamp)/1000000 > 2.5:
            break
        zax += imu.acc_x
        zay += imu.acc_y
        zgz += imu.gyro_z
        cnt += 1
    zax /= cnt
    zay /= cnt
    zgz /= cnt

    px = 0
    py = 0
    theta = 0
    vx = 0
    vy = 0
    omega = 0
    prev_timestamp = imu_data[0].timestamp
    for imu in imu_data:
        delta_t = 1.0*int32(imu.timestamp - prev_timestamp)/1000000
        prev_timestamp = imu.timestamp
        acc_x = (imu.acc_x - zax)*cos(theta) - (imu.acc_y - zay)*sin(theta)
        acc_y = (imu.acc_x - zax)*sin(theta) + (imu.acc_y - zay)*cos(theta)
        px += vx * delta_t + delta_t**2 / 2 * acc_x
        py += vy * delta_t + delta_t**2 / 2 * acc_y
        theta += omega * delta_t + delta_t**2 / 2 *(imu.gyro_z - zgz)
        vx += acc_x * delta_t
        vy += acc_y * delta_t
        omega += (imu.gyro_z - zgz) * delta_t
    print('imu integration: px py theta vx vy omega')
    print(px)
    print(py)
    print(theta)
    print(vx)
    print(vy)
    print(omega)
    return (px, py, theta)

simple_pos_integrate_from_enc_speed(enc_speed_data)
simple_pos_integrate_from_imu(imu_data)



'''
State variables:
pos_x
pos_y
theta
vel_x
vel_y
omega
gyro_z_null
acc_x_null
acc_y_null
imu_orientation
D0
D1
D2
R0
R1
R2

update:
acc
gyro

measurement:
encoder0-2 (wheel speed rad/s)

measurement:
position

'''


def state_transition_fn(s, u):
    state_transition_fn.lastcall
    imu_pos_angle = 50.0 / 180 * pi
    imu_pos_r = 0.742
    n = NastyaState()
    n.update_from_mu(s)
    new = NastyaState()
    delta_t = int32(u.timestamp - state_transition_fn.lastcall) * 1.0 / 1000000
    # transform the acc-measurements onto table coords and consider the
    # estimation of the IMU's orientation and the origin
    acc_x = (  cos(n.imu_orientation + n.theta) * (u.acc_x - n.acc_x_null)
             + sin(n.imu_orientation + n.theta) * (u.acc_y - n.acc_y_null) )
    acc_y = (  sin(n.imu_orientation + n.theta) * (u.acc_x - n.acc_x_null)
             + cos(n.imu_orientation + n.theta) * (u.acc_y - n.acc_y_null) )
    # subtract the acc that is introduced by the IMU not being in the COF
    # imu_pos_angle and imu_pos_r are constants
    acc_x -= cos(n.imu_orientation - imu_pos_angle) * imu_pos_r * n.omega**2
    acc_y -= -sin(n.imu_orientation - imu_pos_angle) * imu_pos_r * n.omega**2

    acc_x_global[0] = acc_x
    acc_x_global[1] = acc_y
    # integrate in tabel coordinates
    new.vel_x = n.vel_x + delta_t * acc_x
    new.vel_y = n.vel_y + delta_t * acc_y
    new.pos_x = n.pos_x + n.vel_x * delta_t + 1/2 * delta_t**2 * acc_x
    new.pos_y = n.pos_y + n.vel_y * delta_t + 1/2 * delta_t**2 * acc_y
    # new.omega = (u.gyro_z - n.gyro_z_null)
    new.theta = n.theta + new.omega * delta_t

    n.vel_x = new.vel_x
    n.vel_y = new.vel_y
    n.pos_x = new.pos_x
    n.pos_y = new.pos_y
    n.omega = new.omega
    n.theta = new.theta

    n.time = u.timestamp

    return n.get_mu()

def z_predict(s):
    ''' vel_x,vel_y,omega,D*,R* --(robot_to_wheel_transform)->  w0,w1,w2
    '''

    nastya = NastyaState()
    nastya.update_from_mu(s)

    beta_0 = 0
    beta_1 = 2/3*pi
    beta_2 = 4/3*pi

    w0 = (-nastya.D0 * nastya.omega +
          sin(beta_0) * nastya.vel_x -
          cos(beta_0) * nastya.vel_y) / nastya.R0
    w1 = (-nastya.D1 * nastya.omega +
          sin(beta_1) * nastya.vel_x -
          cos(beta_1) * nastya.vel_y) / nastya.R1
    w2 = (-nastya.D2 * nastya.omega +
          sin(beta_2) * nastya.vel_x -
          cos(beta_2) * nastya.vel_y) / nastya.R2

    return np.array([[w0], [w1], [w2]])

def z_predict_position(s):
    return np.array([[s.pos_x], [s.pos_y], [s.theta]])


class NastyaState:
    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.theta = 0
        self.vel_x = 0
        self.vel_y = 0
        self.omega = 0
        self.gyro_z_null = -0.01
        self.acc_x_null = 0.743
        self.acc_y_null = 0.05
        self.imu_orientation = 2.7448
        self.D0 = 0.09385
        self.D1 = 0.09385
        self.D2 = 0.09385
        self.R0 = 0.0175
        self.R1 = 0.0175
        self.R2 = 0.0175
        self.time = 0

        cov_vector = np.array([[0.0000001],         # pos_x
                               [0.0000001],         # pos_y
                               [0.0000001],         # theta
                               [0.0000001],         # vel_x
                               [0.0000001],         # vel_y
                               [0.0000001],         # omega
                               [0.01**2],   # gyro_z_null
                               [0.01**2],    # acc_x_null
                               [0.01**2],    # acc_y_null
                               [0.05**2],    # imu_orientation
                               [0.0000000005**2],  # D0
                               [0.0000000005**2],  # D1
                               [0.0000000005**2],  # D2
                               [0.0000000001**2],  # R0
                               [0.0000000001**2],  # R1
                               [0.0000000001**2]]) # R2

        self.cov = np.identity(16) * cov_vector

    def __str__(self):
        return ("pos_x ="
               + str(self.pos_x)
               + "\npos_y ="
               + str(self.pos_y)
               + "\ntheta ="
               + str(self.theta)
               + "\nvel_x ="
               + str(self.vel_x)
               + "\nvel_y ="
               + str(self.vel_y)
               + "\nomega ="
               + str(self.omega)
               + "\ngyro_z_null ="
               + str(self.gyro_z_null)
               + "\nacc_x_null ="
               + str(self.acc_x_null)
               + "\nacc_y_null ="
               + str(self.acc_y_null)
               + "\nimu_orientation ="
               + str(self.imu_orientation)
               + "\nD0 ="
               + str(self.D0)
               + "\nD1 ="
               + str(self.D1)
               + "\nD2 ="
               + str(self.D2)
               + "\nR0 ="
               + str(self.R0)
               + "\nR1 ="
               + str(self.R1)
               + "\nR2 ="
               + str(self.R2))

    def get_mu(self):
        return np.array([[self.pos_x],
                         [self.pos_y],
                         [self.theta],
                         [self.vel_x],
                         [self.vel_y],
                         [self.omega],
                         [self.gyro_z_null],
                         [self.acc_x_null],
                         [self.acc_y_null],
                         [self.imu_orientation],
                         [self.D0],
                         [self.D1],
                         [self.D2],
                         [self.R0],
                         [self.R1],
                         [self.R2]])

    def update_from_mu(self, mu):
        self.pos_x = np.squeeze(mu[0])
        self.pos_y = np.squeeze(mu[1])
        self.theta = np.squeeze(mu[2])
        self.vel_x = np.squeeze(mu[3])
        self.vel_y = np.squeeze(mu[4])
        self.omega = np.squeeze(mu[5])
        self.gyro_z_null = np.squeeze(mu[6])
        self.acc_x_null = np.squeeze(mu[7])
        self.acc_y_null = np.squeeze(mu[8])
        self.imu_orientation = np.squeeze(mu[9])
        self.D0 = np.squeeze(mu[10])
        self.D1 = np.squeeze(mu[11])
        self.D2 = np.squeeze(mu[12])
        self.R0 = np.squeeze(mu[13])
        self.R1 = np.squeeze(mu[14])
        self.R2 = np.squeeze(mu[15])
        return self


from kalman import *

nastya = NastyaState()
state_transition_fn.lastcall = imu_data[0].timestamp
UKF = UnscentedKalmanFilter(nastya.get_mu(),
                            nastya.cov,
                            state_transition_fn,
                            z_predict)

UKF.param.l = 0.5
UKF.param.alpha = 0.5

x_pos = []
x_vel = []
x_acc = []
y_pos = []
y_vel = []
y_acc = []
theta = []
omega = []

for x in data:
    x.kalman(UKF)
    x_pos.append(UKF.mu[0][0])
    x_vel.append(UKF.mu[3][0])
    x_acc.append(acc_x_global[0])
    y_pos.append(UKF.mu[1][0])
    y_vel.append(UKF.mu[4][0])
    y_acc.append(acc_x_global[1])
    theta.append(UKF.mu[2][0])
    omega.append(UKF.mu[5][0])
print("mu =")
print(NastyaState().update_from_mu(UKF.mu))
print("cov =")
print(UKF.cov)

fig = plt.figure()
plt.plot(x_pos)
plt.plot(x_vel)
plt.plot(x_acc)
fig2 = plt.figure()
plt.plot(y_pos)
plt.plot(y_vel)
plt.plot(y_acc)
fig3 = plt.figure()
plt.plot(theta)
plt.plot(omega)
pylab.matshow(UKF.cov, cmap=pylab.cm.gray)
pylab.show()
plt.show()
