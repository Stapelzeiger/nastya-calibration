
from math import sin,cos,pi
from robot_base_mixer import *

class Cmd:
    def __init__(self, ax, ay, alpha, duration):
        self.acc_x = ax
        self.acc_y = ay
        self.alpha = alpha
        self.duration = duration

def run(cmd):
    px = 0
    py = 0
    theta = 0
    vx = 0
    vy = 0
    omega = 0
    t = 0
    output = 0
    imu = []
    enc = []
    pos = []
    for c in cmd:
        delta_t = 0.001
        for k in range(int(c.duration/delta_t)):
            acc_x = c.acc_x - 0.5 * vx
            acc_y = c.acc_y - 0.5 * vy
            alpha = c.alpha - 0.5 * omega
            px += vx * delta_t + 0.5 * acc_x * delta_t**2
            py += vy * delta_t + 0.5 * acc_y * delta_t**2
            theta += omega * delta_t + 0.5 * alpha * delta_t**2
            vx += acc_x * delta_t
            vy += acc_y * delta_t
            omega += alpha * delta_t
            output += 1
            if (output == 10):
                output = 0
                ax_r = cos(theta) * acc_x + sin(theta) * acc_y
                ay_r = -sin(theta) * acc_x + cos(theta) * acc_y
                imu.append([int(t*1000000), ax_r, ay_r, 0, 0, 0, omega])
                vx_r = cos(theta) * vx + sin(theta) * vy
                vy_r = -sin(theta) * vx + cos(theta) * vy
                w0, w1, w2 = robot_to_wheel(vx_r, vy_r, omega)
                ENCODER_RESOLUTION = 16384
                enc.append([int(t*1000000),
                    int(w0*ENCODER_RESOLUTION/2/pi),
                    int(w1*ENCODER_RESOLUTION/2/pi),
                    int(w2*ENCODER_RESOLUTION/2/pi)])
                pos.append([int(t*1000000), px, py, theta])
            t += delta_t
    return imu, enc, pos


imu, enc, pos = run([
        Cmd(1, 0, 0, 0.5),
        Cmd(-1, -1, 0, 0.5),
        Cmd(0, 1, 0, 0.5),
        Cmd(0, 0, 0, 3.5)
        # Cmd(-1, -1, 0, 0.5),
        # Cmd(0, 1, 1, 0.5),
        # Cmd(0, 0, 0, 0),
        # Cmd(0, 0, -1, 0.5),
        # Cmd(-1, 0, 0, 0.5),
        # Cmd(0, 0, 0, 0.5)
    ])

print(pos)


import matplotlib.pyplot as plt

# print()
plt.plot(zip(*pos)[1], zip(*pos)[2])
# plt.plot(zip(*imu)[1])
# plt.plot(zip(*pos)[1])
plt.show()


import csv

imufile = csv.writer(open('data/imu_sim.csv', 'wb'), delimiter=',')
for i in imu:
    imufile.writerow(i)

encfile = csv.writer(open('data/enc_sim.csv', 'wb'), delimiter=',')
for i in enc:
    encfile.writerow(i)