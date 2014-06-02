import numpy as np;
from math import cos, sin, radians;

def euler_angles_to_rotation_matrix(phi, theta, psi, is_degree=True):
    if is_degree:
        phi = radians(phi)
        theta = radians(theta)
        psi = radians(psi)
        
    X = np.array([[1, 0, 0],
                  [0, cos(phi), -1*sin(phi)],
                  [0, sin(phi), cos(phi)]]);
    Y = np.array([[cos(theta), 0, sin(theta)],
                  [0, 1, 0],
                  [-1*sin(theta), 0, cos(theta)]]);
    Z = np.array([[cos(psi), -1*sin(psi), 0],
                  [sin(psi), cos(psi), 0],
                  [0, 0, 1]]);

    return np.dot(np.dot(Z,Y),X)

print euler_angles_to_rotation_matrix(0, 180, 0)

print euler_angles_to_rotation_matrix(90, 0, 90)

def axis_angle_to_quaternion(theta, n, is_degree=True):
    if is_degree:
        theta = radians(theta)
    
    return np.insert(sin(theta/2) * n, 0, cos(theta/2))

print axis_angle_to_quaternion(180, np.array([0,0,1]))