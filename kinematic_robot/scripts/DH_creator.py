from sympy import sin, cos, Abs, symbols, init_printing, Matrix, eye, pi, lambdify, zeros
import numpy as np

print("Kinematic: Start loading inital parameters ...")
# creating joint angles as symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 theta_7')
joint_angles = [q1, q2, q3, q4, q5, q6, q7]

dh_table = [
    {'a':  0,      'd': 0.333, 'alpha': -pi/2},
    {'a':  0,      'd': 0,     'alpha':  pi/2},
    {'a':  0.0825, 'd': 0.316, 'alpha':  pi/2},
    {'a':  -0.0825,'d': 0,     'alpha': -pi/2},
    {'a':  0,      'd': 0.384, 'alpha':  pi/2},
    {'a':  0.088,  'd': 0,     'alpha':  pi/2},
    {'a':  0,      'd': 0.107, 'alpha':  0   },
]

DK = eye(4)

for i, (p, q) in enumerate(zip(reversed(dh_table), reversed(joint_angles))):
    d = p['d']
    a = p['a']
    alpha = p['alpha']

    ca = cos(alpha)
    sa = sin(alpha)
    cq = cos(q)
    sq = sin(q)

    transform = Matrix([
        
            [cq, -ca*sq, sa*sq, a*cq],
            [sq, ca * cq, -sa*cq, a * sq],
            [0, sa, ca, d ],
            [0, 0, 0, 1],   
    ])

    DK = transform @ DK


A = DK[0:3, 0:4]  # croping last row

A = A.transpose().reshape(12,1)  # reshaping to column vector A = [a11, a21, a31, ..., a34]
Q = Matrix(joint_angles)
J = A.jacobian(Q)  # computing the Jacobian symbolically

#print(A)
print(J)
