from kalman import *
import numpy as np


def trans(mu, u):
    return mu + u

def h(mu):
    return mu

mu0 = np.array([[0], [0]])
cov0 = np.array([[1, 0], [0, 1]])
control = np.array([[1], [0]])
measurement = np.array([[0.9], [0.1]])
R = np.array([[0.2, 0], [0, 0.2]])
Q = np.array([[0.05, 0], [0, 0.05]])

UKF = UnscentedKalmanFilter(mu0, cov0, trans, h)
UKF.param.l = 0.5
UKF.param.alpha = 0.5

print("mu0 =")
print(mu0)
print("cov0 =")
print(cov0)

print('')
print("control")
print("updating with:")
print("u =")
print(control)
print("R =")
print(R)
UKF.control_update(control, R)
print("after:")
print("mu =")
print(UKF.mu)
print("cov =")
print(UKF.cov)

print('')
print("measurement:")
print("updating with:")
print("z =")
print(measurement)
print("Q =")
print(Q)
UKF.measurement_update(measurement, Q=Q)
print("after:")
print("mu =")
print(UKF.mu)
print("cov =")
print(UKF.cov)
