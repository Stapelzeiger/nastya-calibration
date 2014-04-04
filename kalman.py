from __future__ import division

from math import *
import numpy as np
from numpy import linalg

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class UnscentedTransformParam:
    def __init__(self, l=1, alpha=1, beta=1):
        self.l = l
        self.alpha = alpha
        self.beta = beta

def get_sigma_pts(mu, cov_matrix, param=UnscentedTransformParam()):
    n = len(mu)
    L = linalg.cholesky(cov_matrix)
    scaling = sqrt(n + param.l)
    sigma_pts = [mu]
    sigma_pts += [mu + np.array([column]).T * scaling for column in L.T]
    sigma_pts += [mu - np.array([column]).T * scaling for column in L.T]
    return sigma_pts

def get_mu(sigma_pts, param=UnscentedTransformParam()):
    n = len(sigma_pts[0])
    print(n)
    mu = sigma_pts[0] * param.l/(n + param.l)
    print(mu)
    mu += sum([s * 1/(2 * (n + param.l)) for s in sigma_pts[1:]])
    return mu

def get_cov_matrix(sigma_pts, mu, param=UnscentedTransformParam()):
    n = len(mu)
    w0 = param.l/(n + param.l) + (1 - param.alpha**2 + param.beta)
    cov = w0 * np.dot((sigma_pts[0] - mu), (sigma_pts[0] - mu).T)
    wi = 1/(2 * (n + param.l))
    cov += sum([wi * np.dot((s - mu), (s - mu).T) for s in sigma_pts[1:]])
    return cov


mu = np.array([[1], [2], [3]])

cov = np.array([[10, 5, 0],
                [5, 10, 0],
                [0, 0, 0.001]])


def test_unscented_transf():
    # mu = np.array([[1], [2]])

    # cov = np.array([[5, 0],
    #                 [0, 4]])

    print('mu')
    print(mu)
    print('cov')
    print(cov)
    sigma_pts = get_sigma_pts(mu, cov)
    print('sigma points')
    for s in sigma_pts:
        print(s)
        print('')

    mup = get_mu(sigma_pts)
    print(mup)
    covp = get_cov_matrix(sigma_pts, mup)
    print(covp)

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(*zip(*sigma_pts))
    # plt.show()


class UnscentedKalmanFilter:
    def __init__(self, mu0, cov0, transistion, z_predict):
        self.mu = mu0
        self.cov = cov0
        self.transistion = transistion
        self.default_z_predict = z_predict
        self.param = UnscentedTransformParam()

    def control_update(self, u, R=None):
        sigma_pts = get_sigma_pts(self.mu, self.cov, self.param)
        sigma_pts = [self.transistion(s, u) for s in sigma_pts]
        self.mu = get_mu(sigma_pts, self.param)
        self.cov = get_cov_matrix(sigma_pts, self.mu, self.param)
        if R:
            self.cov += R
        return (self.mu, self.cov)

    def measurement_update(self, z, z_predict=None, Q=None):
        def get_kalman_gain(sigma_pts, mu, z_pts, mu_z, S, param):
            n = len(mu)
            w0 = param.l/(n + param.l) + (1 - param.alpha**2 + param.beta)
            tmp = w0 * np.dot((sigma_pts[0] - mu), (z_pts[0] - mu_z).T)
            wi = 1/(2 * (n + param.l))
            Sigma_x_z_t += sum([wi * np.dot((s - mu), (z - mu_z).T)
                    for s, z in zip(sigma_pts[1:], z_pts[1:])])
            return np.dot(Sigma_x_z_t, linalg.inv(S))

        if not z_predict:
            z_predict = self.default_z_predict
        sigma_pts = get_sigma_pts(self.mu, self.cov, self.param)

        z_pts = [z_predict(s) for s in sigma_pts]
        mu_z = [get_mu(z_pts) for z in z_pts]
        S = get_cov_matrix(z_pts, mu_z, self.param)
        if Q:
            S += Q
        K = get_kalman_gain(sigma_pts, self.mu, z_pts, mu_z, S, self.param)
        self.mu = self.mu + np.dot(K, (z - mu_z))
        self.cov = self.cov - np.dot(np.dot(K, S), K.T)
