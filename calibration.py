import numpy as np
from scipy.optimize import minimize
from math import *


def rotate_vect(vect, ang):
    return np.matrix([[cos(ang), -sin(ang)], [sin(ang), cos(ang)]]) * vect


# pos, a, b, c: 2d points (np.matrix); orientation: angle
def robot_orientation_and_position_err_fn(pos, orientation, a, b, c):
    ref_a = np.matrix([[1], [0]])
    ref_b = np.matrix([[0], [0]])
    ref_c = np.matrix([[0], [1]])

    ref_ap = pos + rotate_vect(ref_a, orientation)
    ref_bp = pos + rotate_vect(ref_b, orientation)
    ref_cp = pos + rotate_vect(ref_c, orientation)
    print("abc:")
    print(ref_ap)
    print(ref_bp)
    print(ref_cp)

    err = (np.linalg.norm(ref_ap - a)**2
         + np.linalg.norm(ref_bp - b)**2
         + np.linalg.norm(ref_cp - c)**2)
    print("err:")
    print(err)
    return err


def find_robot_orientation_and_position(a, b, c):
    x0 = np.array([0, 0, 0])
    return minimize(lambda x: robot_orientation_and_position_err_fn(np.matrix([[x[0]], [x[1]]]), x[2] , a, b, c), x0)


# print(rotate_vect(np.matrix([[1], [0]]), 3.1416/2))

# print(find_robot_orientation_and_position(np.matrix([4, -1]), np.matrix([4, 0]), np.matrix([5, 0])))

print(find_robot_orientation_and_position(np.matrix([2, -3]), np.matrix([1, -3]), np.matrix([1, -2])))


# print(find_robot_orientation_and_position(np.matrix([0, -1])+np.matrix([2, -3]), np.matrix([0, 0])+np.matrix([2, -3]), np.matrix([1, 0])+np.matrix([2, -3])))

