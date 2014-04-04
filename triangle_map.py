import numpy as np
from math import *

def get_centroid(a, b, c):
    return (a + b + c) / 3


def get_angle_of_line(a, b):
    diff = b - a
    return np.arctan2(diff[1], diff[0])

def rotate_vector(vect, ang):
    return np.matrix([[cos(ang), -sin(ang)], [sin(ang), cos(ang)]]) * vect


def calculate_position(a, b, c):
    ''' a, b, c are the corners of the triangle.
        ref_x has to be set to the corresponding corner of the reference
        (delta to center of robot)
    '''
    ref_a = np.matrix([[1], [0]])
    ref_b = np.matrix([[0], [0]])
    ref_c = np.matrix([[0], [1]])

    ref_centroid = get_centroid(ref_a, ref_b, ref_c)

    print("ref_centroid")
    print(ref_centroid)

    ref_angle_0 = get_angle_of_line(ref_a, ref_b)
    ref_angle_1 = get_angle_of_line(ref_b, ref_c)
    ref_angle_2 = get_angle_of_line(ref_c, ref_a)

    angle_0 = get_angle_of_line(a, b)
    angle_1 = get_angle_of_line(b, c)
    angle_2 = get_angle_of_line(c, a)

    print(ref_angle_0)
    print(ref_angle_1)
    print(ref_angle_2)

    mean_orientation = (angle_0 + angle_1 + angle_2
                         - ref_angle_0 - ref_angle_1 - ref_angle_2) / 3
    print("rotated")
    print(rotate_vector(ref_centroid, mean_orientation))
    position = get_centroid(a, b, c) -
            rotate_vector(ref_centroid, mean_orientation)

    return position, mean_orientation
