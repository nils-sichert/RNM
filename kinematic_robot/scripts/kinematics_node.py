#!/usr/bin/env python3
import numpy as np
from sympy import symbols, init_printing, Matrix, eye, sin, cos, pi, lambdify
init_printing(use_unicode=True)

# creating joint angles as symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 theta_7')
joint_angles = [q1, q2, q3, q4, q5, q6, q7]

# defining joint limits for the Panda robot
limits = [
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973)
]

# panda robot dh_table
dh_table = [
    {'a':  0,      'd': 0.333, 'alpha':  0,  },
    {'a':  0,      'd': 0,     'alpha': -pi/2},
    {'a':  0,      'd': 0.316, 'alpha':  pi/2},
    {'a':  0.0825, 'd': 0,     'alpha':  pi/2},
    {'a': -0.0825, 'd': 0.384, 'alpha': -pi/2},
    {'a':  0,      'd': 0,     'alpha':  pi/2},
    {'a':  0.088,  'd': 0.107, 'alpha':  pi/2},
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

    transform = Matrix(
        [
            [cq, -sq, 0, a],
            [ca * sq, ca * cq, -sa, -d * sa],
            [sa * sq, cq * sa, ca, d * ca],
            [0, 0, 0, 1],
        ]
    )

    DK = transform @ DK

DK.evalf(subs={
    'theta_1': 0,
    'theta_2': 0,
    'theta_3': 0,
    'theta_4': 0,
    'theta_5': 0,
    'theta_6': 0,
    'theta_7': 0,
})

A = DK[0:3, 0:4]  # croping last row
A = A.transpose().reshape(12,1)  # reshaping to column vector A = [a11, a21, a31, ..., a34]

Q = Matrix(joint_angles)
J = A.jacobian(Q)  # computing the Jacobian symbolically

# converting symbolic form into lambda funtion
A_lamb = (lambdify((q1, q2, q3, q4, q5, q6, q7), A, 'numpy'))
J_lamb = (lambdify((q1, q2, q3, q4, q5, q6, q7), J, 'numpy'))


# create initial pose
q_init = np.array([l+(u-l)/2 for l, u in limits], dtype=np.float64).reshape(7, 1)
A_init = A_lamb(*(q_init.flatten()))
print(A_init.reshape(3, 4, order='F'))

# generating a random final pose within joint limits
np.random.seed(0)
q_rand = np.array([np.random.uniform(l, u) for l, u in limits], dtype=np.float64).reshape(7, 1)
A_final = A_lamb(*(q_rand).flatten())
print(A_final.reshape(3, 4, order='F'))

# building the incremental inverse kinematics
def incremental_ik(q, A, A_final, step=0.1, atol=1e-4):
    while True:
        delta_A = (A_final - A)
        if np.max(np.abs(delta_A)) <= atol:
            break
        J_q = J_lamb(q[0,0], q[1,0], q[2,0], q[3,0], q[4,0], q[5,0], q[6,0])
        J_q = J_q / np.linalg.norm(J_q)  # normalizing the jacobian
        
        # multiplying by step_size to interpolate between current and target pose
        delta_q = np.linalg.pinv(J_q) @ (delta_A*step)
        
        q = q + delta_q
        A = A_lamb(q[0,0], q[1,0],q[2,0],q[3,0],q[4,0],q[5,0],q[6,0])
    return q, np.max(np.abs(delta_A))


q_final, _ = incremental_ik(q_init, A_init, A_final, atol=1e-6)
q_final.flatten()