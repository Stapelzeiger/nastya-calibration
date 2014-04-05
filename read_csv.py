import csv

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
    def __repr__(self):
        return 'encspeed@{}'.format(self.timestamp)

class IMUData:
    def __init__(self, csv_row):
        self.timestamp = int(csv_row[0])
        self.accx = float(csv_row[1])
        self.accy = float(csv_row[2])
        self.accz = float(csv_row[3])
        self.gyrox = float(csv_row[4])
        self.gyroy = float(csv_row[5])
        self.gyroz = float(csv_row[6])
    def __repr__(self):
        return 'imu@{}'.format(self.timestamp)

def read_csv(csv_file, constr):
    print(csv_file)
    data = []
    for line, row in enumerate(csv.reader(open(csv_file), delimiter=',')):
        try:
            data.append(constr(row))
        except Exception, e:
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


print(sorted((enc_speed_data + imu_data), key=lambda x: x.timestamp))



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

import numpy as np
from math import *

def state_transition_fn(s, u):
    '''
        # transform the acc-measurements onto table coords and consider the
        # estimation of the IMU's orientation and the origin
        acc_x = (  cos(s.imu_orientation + s.theta) * (u.acc_x - s.acc_x_null)
                 + sin(s.imu_orientation + s.theta) * (u.acc_y - s.acc_y_null) )
        acc_y = (  sin(s.imu_orientation + s.theta) * (u.acc_x - s.acc_x_null)
                 + cos(s.imu_orientation + s.theta) * (u.acc_y - s.acc_y_null) )
        # subtract the acc that is introduced by the IMU not being in the COF
        # imu_pos_angle and imu_pos_r are constants
        acc_x -= cos(s.imu_orientation - imu_pos_angle) * imu_pos_r * omega**2
        acc_y -= -sin(s.imu_orientation - imu_pos_angle) * imu_pos_r * omega**2
        # integrate in tabel coordinates
        new.vel_x += u.delta_t * acc_x
        new.vel_y += u.delta_t * acc_y
        new.pos_x += s.vel_x * u.delta_t + 1/2 * u.delta_t**2 * acc_x
        new.pos_y += s.vel_y * u.delta_t + 1/2 * u.delta_t**2 * acc_y
        new.omega = (u.gyro_z - s.gyro_z_null)
        new.theta += new.omega * u.delta_t
    '''

def z_predict(s):
    ''' vel_x,vel_y,omega,D*,R* --(robot_to_wheel_transform)->  w0,w1,w2
    '''
        # these constants are probably wrong...
        beta_0 = 0
        beta_1 = 2/3*pi
        beta_2 = 4/3*pi

        w0 = (-s.D0 * s.omega + sin(beta_0) * s.vel_x - cos(beta_0) * s.vel_y) / s.R0
        w1 = (-s.D1 * s.omega + sin(beta_1) * s.vel_x - cos(beta_1) * s.vel_y) / s.R1
        w2 = (-s.D2 * s.omega + sin(beta_2) * s.vel_x - cos(beta_2) * s.vel_y) / s.R2

        return np.array([[w0], [w1], [w2]])
