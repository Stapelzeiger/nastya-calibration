import csv
import numpy as np
from math import *

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
        self.timestamp = b.timestamp
        self.delta_t = b.timestamp - a.timestamp
        self.w0 = int32(b.enc0 - a.enc0)/self.delta_t
        self.w1 = int32(b.enc1 - a.enc1)/self.delta_t
        self.w2 = int32(b.enc2 - a.enc2)/self.delta_t

        Q_vector = np.array([[1],
                             [1],
                             [1]])

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

        R_vector = np.array([[1],
                             [1],
                             [1],
                             [1],
                             [1],
                             [1]])

        self.R = np.identity(6) * R_vector

    def __repr__(self):
        return 'imu@{}'.format(self.timestamp)

    def kalman(self, kalman_filter):
        kalman_filter.control_update(self, self.R)


def read_csv(csv_file, constr):
    print(csv_file)
    data = []
    for line, row in enumerate(csv.reader(open(csv_file), delimiter=',')):
        try:
            data.append(constr(row))
        except Exception as e:
            print('invalid datapoint line: ' + str(line+1) + ' (' + str(e) + ')')
    return data

enc_data = sorted(read_csv('data/enc_test.csv', EncData), key=lambda x: x.timestamp)
imu_data = sorted(read_csv('data/imu_test.csv', IMUData), key=lambda x: x.timestamp)

enc_speed_data = []
for i in range(len(enc_data) - 1):
    enc_speed_data.append(EncSpeed(enc_data[i], enc_data[i+1]))

print(enc_data)
print(imu_data)
print(enc_speed_data)


data = sorted((enc_speed_data + imu_data), key=lambda x: x.timestamp)
print(data)



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
    imu_pos_angle = 0
    imu_pos_r = 0
    n = NastyaState()
    n.update_from_mu(s)
    new = NastyaState()
    delta_t = u.timestamp - n.time
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
    # integrate in tabel coordinates
    new.vel_x += delta_t * acc_x
    new.vel_y += delta_t * acc_y
    new.pos_x += n.vel_x * delta_t + 1/2 * delta_t**2 * acc_x
    new.pos_y += n.vel_y * delta_t + 1/2 * delta_t**2 * acc_y
    new.omega = (u.gyro_z - n.gyro_z_null)
    new.theta += new.omega * delta_t

    n.vel_x = new.vel_x
    n.vel_y = new.vel_y
    n.pos_x = new.pos_x
    n.pos_y = new.pos_y
    n.omega = new.omega
    n.theta = new.theta

    n.time = u.timestamp

    s = n.get_mu()

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

    return np.array([w0, w1, w2])

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
        self.gyro_z_null = 0
        self.acc_x_null = 0
        self.acc_y_null = 0
        self.imu_orientation = 0
        self.D0 = 0.09385
        self.D1 = 0.09385
        self.D2 = 0.09385
        self.R0 = 0.0175
        self.R1 = 0.0175
        self.R2 = 0.0175
        self.time = 0

        cov_vector = np.array([[1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1],
                               [1]])

        self.cov = np.identity(16) * cov_vector

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
        self.pos_x = mu[0]
        self.pos_y = mu[1]
        self.theta = mu[2]
        self.vel_x = mu[3]
        self.vel_y = mu[4]
        self.omega = mu[5]
        self.gyro_z_null = mu[6]
        self.acc_x_null = mu[7]
        self.acc_y_null = mu[8]
        self.imu_orientation = mu[9]
        self.D0 = mu[10]
        self.D1 = mu[11]
        self.D2 = mu[12]
        self.R0 = mu[13]
        self.R1 = mu[14]
        self.R2 = mu[15]
