from math import sin,cos

default_beta = [0,                        #   0 deg
                2.0943951023931954923084, # 120 deg
                4.1887902047863909846168] # 240 deg
default_D = [0.09385,  # inner 87.7 mm outer 100 mm
             0.09385,
             0.09385]
default_r = [0.0175,
             0.0175,
             0.0175]

def wheel_to_robot(dwheels, beta=default_beta, D=default_D, r=default_r):
    c = 1.0 / (  D[0] * sin(beta[1] - beta[2])
               - D[1] * sin(beta[0] - beta[2])
               + D[2] * sin(beta[0] - beta[1]) )
    wtr = [[0,0,0],[0,0,0],[0,0,0]]
    wtr[0][0] = c * sin(beta[2] - beta[1])
    wtr[0][1] = c * sin(beta[0] - beta[2])
    wtr[0][2] = c * sin(beta[1] - beta[0])
    wtr[1][0] = c * (D[2]*cos(beta[1]) - D[1]*cos(beta[2]))
    wtr[1][1] = c * (D[0]*cos(beta[2]) - D[2]*cos(beta[0]))
    wtr[1][2] = c * (D[1]*cos(beta[0]) - D[0]*cos(beta[1]))
    wtr[2][0] = c * (D[2]*sin(beta[1]) - D[1]*sin(beta[2]))
    wtr[2][1] = c * (D[0]*sin(beta[2]) - D[2]*sin(beta[0]))
    wtr[2][2] = c * (D[1]*sin(beta[0]) - D[0]*sin(beta[1]))
    rdwheel = [dwheels[0] * r[0],
               dwheels[1] * r[1],
               dwheels[2] * r[2]]
    dtheta  = (rdwheel[0] * wtr[0][0]
            +  rdwheel[1] * wtr[0][1]
            +  rdwheel[2] * wtr[0][2])
    dx      = (rdwheel[0] * wtr[1][0]
            +  rdwheel[1] * wtr[1][1]
            +  rdwheel[2] * wtr[1][2])
    dy      = (rdwheel[0] * wtr[2][0]
            +  rdwheel[1] * wtr[2][1]
            +  rdwheel[2] * wtr[2][2])
    return (dx, dy, dtheta)