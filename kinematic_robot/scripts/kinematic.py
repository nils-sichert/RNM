#!/usr/bin/env python3
from sympy import sin, cos, Abs, symbols, init_printing, Matrix, eye, pi, lambdify, zeros
import numpy as np
from rospy import Subscriber




class kinematic:
    def __init__(self):
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
        self.A = A.transpose().reshape(12,1)  # reshaping to column vector A = [a11, a21, a31, ..., a34]
        Q = Matrix(joint_angles)
        self.J = self.A.jacobian(Q)  # computing the Jacobian symbolically
        
        # converting symbolic form into lambda funtion
        

        # create initial pose
        #/TODO Subscriber aktuelle POSE
        #self.q_init = np.array(WINKEL)
        #self.A_init = np.array(MATRIX A)

        # defining joint limits for the Panda robot
        self.limits = [
            (-2.8973, 2.8973),
            (-1.7628, 1.7628),
            (-2.8973, 2.8973),
            (-3.0718, -0.0698),
            (-2.8973, 2.8973),
            (-0.0175, 3.7525),
            (-2.8973, 2.8973)
        ]
        print("Kinematic: Initial parameters loaded.")

    def direct_kinematic(self,q):
        A_Mat = self.A.evalf(subs={'theta_1': q[0],'theta_2':  q[1],'theta_3':  q[2],'theta_4':  q[3],'theta_5':  q[4],'theta_6':  q[5],'theta_7':  q[6]})
        position = A_Mat[9:12]
        orientation = np.reshape(A_Mat[0:9], (3,3))

        return position, orientation, A_Mat
    
    def inverse_kinematic(self, q, A_current, A_target, step=0.01, err_Tol=1e-2):
        # input: q = current thetas, ... /TODO
        delta_A = (A_target-A_current)
        while max(Abs(delta_A)) > 0.01:
            delta_A = (A_target - A_current)
            J_q = self.J.evalf(subs={'theta_1': q[0],'theta_2':  q[1],'theta_3':  q[2],'theta_4':  q[3],'theta_5':  q[4],'theta_6':  q[5],'theta_7':  q[6]})
            
            # multiplying by step_size to interpolate between current and target pose
            # TODO need a clean up!
            delta_q = J_q.pinv() @ delta_A*step
            q = q + delta_q
            A_current = self.A.evalf(subs={'theta_1': q[0],'theta_2':  q[1],'theta_3':  q[2],'theta_4':  q[3],'theta_5':  q[4],'theta_6':  q[5],'theta_7':  q[6]})
            delta_A = (A_target - A_current)
        return q
        #/TODO solve function and algorithm



kinemati = kinematic()
theta_current = Matrix([1,1,1,1,1,1,1])
pos, orient, a = kinemati.direct_kinematic(theta_current)
A_current = a
theta_target = Matrix([0,0,0,0,0,0,0])
pos_target, orient_target, a_target = kinemati.direct_kinematic(theta_target)
A_target = a_target
print(A_target)
q_ik = kinemati.inverse_kinematic(theta_current, A_current, A_target)
print(q_ik)