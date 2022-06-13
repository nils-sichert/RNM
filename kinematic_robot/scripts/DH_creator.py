from sympy import sin, cos, Abs, symbols, init_printing, Matrix, eye, pi, lambdify, zeros
from sympy.utilities.lambdify import lambdastr
import numpy as np

print("Kinematic: Start loading inital parameters ...")
# creating joint angles as symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 theta_7')
joint_angles = [q1, q2, q3, q4, q5, q6, q7]

dh_table = [
    {'a':  0,      'd': 0.3330, 'alpha': -pi/2},
    {'a':  0,      'd': 0,      'alpha':  pi/2},
    {'a':  0.0825, 'd': 0.3160, 'alpha':  pi/2},
    {'a': -0.0825, 'd': 0,      'alpha': -pi/2},
    {'a':  0,      'd': 0.3840, 'alpha':  pi/2},
    {'a':  0.0880, 'd': 0,      'alpha':  pi/2},
    {'a':  0,      'd': 0.1070, 'alpha':  0   },
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

        [cq, -ca * sq,  sa * sq, a * cq],
        [sq,  ca * cq, -sa * cq, a * sq],
        [ 0,       sa,       ca,      d],
        [ 0,        0,        0,      1],   
    ])

    DK = transform @ DK


A = DK[0:3, 0:4]  # cropping last row

A = A.transpose().reshape(12,1)  # reshaping to column vector A = [a11, a21, a31, ..., a34]
Q = Matrix(joint_angles)
J = A.jacobian(Q)  # computing the Jacobian symbolically


# Replace cos/sin terms
c1, c2, c3, c4, c5, c6, c7 = symbols('c1, c2, c3, c4, c5, c6, c7')
s1, s2, s3, s4, s5, s6, s7 = symbols('s1, s2, s3, s4, s5, s6, s7')
joint_cos = [c1, c2, c3, c4, c5, c6, c7]
joint_sin = [s1, s2, s3, s4, s5, s6, s7]

for (q, c, s) in zip(joint_angles, joint_cos, joint_sin):
    A = A.subs(cos(q), c)
    A = A.subs(sin(q), s)
    J = J.subs(cos(q), c)
    J = J.subs(sin(q), s)

#print(A)
#print(J)

# Lambdify
A_lamb  = lambdify([(c1, c2, c3, c4, c5, c6, c7), (s1, s2, s3, s4, s5, s6, s7)], A, 'numpy')
J_lamb  = lambdify([(c1, c2, c3, c4, c5, c6, c7), (s1, s2, s3, s4, s5, s6, s7)], J, 'numpy')


# Test
theta_target = np.array([np.deg2rad(5), np.deg2rad(5), np.deg2rad(5), np.deg2rad(5), np.deg2rad(5), np.deg2rad(5),np.deg2rad(5)])
cos_target  = [np.cos(q) for q in theta_target]
sin_target  = [np.sin(q) for q in theta_target]

A_lamb(cos_target, sin_target)
J_lamb(cos_target, sin_target)

pass


A_lamb_explicit = lambda _0,_1: (lambda c1,c2,c3,c4,c5,c6,c7,s1,s2,s3,s4,s5,s6,s7: (np.array([[c1*(c2*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(c5*s7 - c6*c7*s5)) + s2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7))) - s1*(-c3*(c5*s7 - c6*c7*s5) + s3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6))], [c1*(-c3*(c5*s7 - c6*c7*s5) + s3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6)) + s1*(c2*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(c5*s7 - c6*c7*s5)) + s2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7)))], [c2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7)) - s2*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(c5*s7 - c6*c7*s5))], [c1*(c2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7)) + s2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5))) - s1*(-c3*(c5*c7 + c6*s5*s7) + s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7))], [c1*(-c3*(c5*c7 + c6*s5*s7) + s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7)) + s1*(c2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7)) + s2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5)))], [c2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5)) - s2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7))], [c1*(c2*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6) + s2*(-c4*c6 + c5*s4*s6)) - s1*(c3*s5*s6 + s3*(c4*c5*s6 + c6*s4))], [c1*(c3*s5*s6 + s3*(c4*c5*s6 + c6*s4)) + s1*(c2*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6) + s2*(-c4*c6 + c5*s4*s6))], [c2*(-c4*c6 + c5*s4*s6) - s2*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6)], [c1*(c2*(c3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.107*s6)) + s2*(c4*(-0.107*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.107*s6) - 0.0825*s4 + 0.316)) - s1*(c3*s5*(0.088*c6 + 0.107*s6) + s3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*s3)], [c1*(c3*s5*(0.088*c6 + 0.107*s6) + s3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*s3) + s1*(c2*(c3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.107*s6)) + s2*(c4*(-0.107*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.107*s6) - 0.0825*s4 + 0.316))], [c2*(c4*(-0.107*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.107*s6) - 0.0825*s4 + 0.316) - s2*(c3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.107*s6)) + 0.333]])))(_0[0],_0[1],_0[2],_0[3],_0[4],_0[5],_0[6],_1[0],_1[1],_1[2],_1[3],_1[4],_1[5],_1[6])
J_lamb_explicit = lambda _0,_1: (lambda c1,c2,c3,c4,c5,c6,c7,s1,s2,s3,s4,s5,s6,s7: (np.array([[c1*(c3*(c5*s7 - c6*c7*s5) - s3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6)) - s1*(c2*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(c5*s7 - c6*c7*s5)) + s2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7))), c1*(c2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7)) - s2*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(c5*s7 - c6*c7*s5))), c1*c2*(c3*(c5*s7 - c6*c7*s5) - s3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6)) + s1*(-c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(-c5*s7 + c6*c7*s5)), c1*(c2*c3*(-c4*c7*s6 - s4*(c5*c6*c7 + s5*s7)) + s2*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6)) - s1*s3*(-c4*c7*s6 - s4*(c5*c6*c7 + s5*s7)), c1*(c2*(c3*c4*(c5*s7 - c6*c7*s5) + s3*(-c5*c6*c7 - s5*s7)) + s2*s4*(c5*s7 - c6*c7*s5)) + s1*(-c3*(c5*c6*c7 + s5*s7) - c4*s3*(c5*s7 - c6*c7*s5)), c1*(c2*(c3*(-c4*c5*c7*s6 - c6*c7*s4) + c7*s3*s5*s6) + s2*(c4*c6*c7 - c5*c7*s4*s6)) + s1*(c3*c7*s5*s6 - s3*(-c4*c5*c7*s6 - c6*c7*s4)), c1*(c2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7)) + s2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5))) + s1*(-c3*(-c5*c7 - c6*s5*s7) - s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7))], [c1*(c2*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(c5*s7 - c6*c7*s5)) + s2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7))) - s1*(-c3*(c5*s7 - c6*c7*s5) + s3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6)), s1*(c2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7)) - s2*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) + s3*(c5*s7 - c6*c7*s5))), c1*(c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) - s3*(-c5*s7 + c6*c7*s5)) + c2*s1*(c3*(c5*s7 - c6*c7*s5) - s3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6)), c1*s3*(-c4*c7*s6 - s4*(c5*c6*c7 + s5*s7)) + s1*(c2*c3*(-c4*c7*s6 - s4*(c5*c6*c7 + s5*s7)) + s2*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6)), c1*(c3*(c5*c6*c7 + s5*s7) + c4*s3*(c5*s7 - c6*c7*s5)) + s1*(c2*(c3*c4*(c5*s7 - c6*c7*s5) + s3*(-c5*c6*c7 - s5*s7)) + s2*s4*(c5*s7 - c6*c7*s5)), c1*(-c3*c7*s5*s6 + s3*(-c4*c5*c7*s6 - c6*c7*s4)) + s1*(c2*(c3*(-c4*c5*c7*s6 - c6*c7*s4) + c7*s3*s5*s6) + s2*(c4*c6*c7 - c5*c7*s4*s6)), c1*(c3*(-c5*c7 - c6*s5*s7) + s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7)) + s1*(c2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7)) + s2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5)))], [0, c2*(-c3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) - s3*(c5*s7 - c6*c7*s5)) - s2*(c4*c7*s6 + s4*(c5*c6*c7 + s5*s7)), s2*(-c3*(c5*s7 - c6*c7*s5) + s3*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6)), c2*(c4*(c5*c6*c7 + s5*s7) - c7*s4*s6) - c3*s2*(-c4*c7*s6 - s4*(c5*c6*c7 + s5*s7)), c2*s4*(c5*s7 - c6*c7*s5) + s2*(-c3*c4*(c5*s7 - c6*c7*s5) - s3*(-c5*c6*c7 - s5*s7)), c2*(c4*c6*c7 - c5*c7*s4*s6) + s2*(-c3*(-c4*c5*c7*s6 - c6*c7*s4) - c7*s3*s5*s6), c2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5)) + s2*(-c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) - s3*(c5*c7 + c6*s5*s7))], [c1*(c3*(c5*c7 + c6*s5*s7) - s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7)) - s1*(c2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7)) + s2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5))), c1*(c2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5)) - s2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7))), c1*c2*(c3*(c5*c7 + c6*s5*s7) - s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7)) + s1*(-c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(-c5*c7 - c6*s5*s7)), c1*(c2*c3*(c4*s6*s7 - s4*(-c5*c6*s7 + c7*s5)) + s2*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7)) - s1*s3*(c4*s6*s7 - s4*(-c5*c6*s7 + c7*s5)), c1*(c2*(c3*c4*(c5*c7 + c6*s5*s7) + s3*(c5*c6*s7 - c7*s5)) + s2*s4*(c5*c7 + c6*s5*s7)) + s1*(-c3*(-c5*c6*s7 + c7*s5) - c4*s3*(c5*c7 + c6*s5*s7)), c1*(c2*(c3*(c4*c5*s6*s7 + c6*s4*s7) - s3*s5*s6*s7) + s2*(-c4*c6*s7 + c5*s4*s6*s7)) + s1*(-c3*s5*s6*s7 - s3*(c4*c5*s6*s7 + c6*s4*s7)), c1*(c2*(c3*(c4*(-c5*c6*c7 - s5*s7) + c7*s4*s6) + s3*(-c5*s7 + c6*c7*s5)) + s2*(-c4*c7*s6 + s4*(-c5*c6*c7 - s5*s7))) + s1*(-c3*(c5*s7 - c6*c7*s5) - s3*(c4*(-c5*c6*c7 - s5*s7) + c7*s4*s6))], [c1*(c2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7)) + s2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5))) - s1*(-c3*(c5*c7 + c6*s5*s7) + s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7)), s1*(c2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5)) - s2*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) + s3*(c5*c7 + c6*s5*s7))), c1*(c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) - s3*(-c5*c7 - c6*s5*s7)) + c2*s1*(c3*(c5*c7 + c6*s5*s7) - s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7)), c1*s3*(c4*s6*s7 - s4*(-c5*c6*s7 + c7*s5)) + s1*(c2*c3*(c4*s6*s7 - s4*(-c5*c6*s7 + c7*s5)) + s2*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7)), c1*(c3*(-c5*c6*s7 + c7*s5) + c4*s3*(c5*c7 + c6*s5*s7)) + s1*(c2*(c3*c4*(c5*c7 + c6*s5*s7) + s3*(c5*c6*s7 - c7*s5)) + s2*s4*(c5*c7 + c6*s5*s7)), c1*(c3*s5*s6*s7 + s3*(c4*c5*s6*s7 + c6*s4*s7)) + s1*(c2*(c3*(c4*c5*s6*s7 + c6*s4*s7) - s3*s5*s6*s7) + s2*(-c4*c6*s7 + c5*s4*s6*s7)), c1*(c3*(c5*s7 - c6*c7*s5) + s3*(c4*(-c5*c6*c7 - s5*s7) + c7*s4*s6)) + s1*(c2*(c3*(c4*(-c5*c6*c7 - s5*s7) + c7*s4*s6) + s3*(-c5*s7 + c6*c7*s5)) + s2*(-c4*c7*s6 + s4*(-c5*c6*c7 - s5*s7)))], [0, c2*(-c3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) - s3*(c5*c7 + c6*s5*s7)) - s2*(-c4*s6*s7 + s4*(-c5*c6*s7 + c7*s5)), s2*(-c3*(c5*c7 + c6*s5*s7) + s3*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7)), c2*(c4*(-c5*c6*s7 + c7*s5) + s4*s6*s7) - c3*s2*(c4*s6*s7 - s4*(-c5*c6*s7 + c7*s5)), c2*s4*(c5*c7 + c6*s5*s7) + s2*(-c3*c4*(c5*c7 + c6*s5*s7) - s3*(c5*c6*s7 - c7*s5)), c2*(-c4*c6*s7 + c5*s4*s6*s7) + s2*(-c3*(c4*c5*s6*s7 + c6*s4*s7) + s3*s5*s6*s7), c2*(-c4*c7*s6 + s4*(-c5*c6*c7 - s5*s7)) + s2*(-c3*(c4*(-c5*c6*c7 - s5*s7) + c7*s4*s6) - s3*(-c5*s7 + c6*c7*s5))], [c1*(-c3*s5*s6 - s3*(c4*c5*s6 + c6*s4)) - s1*(c2*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6) + s2*(-c4*c6 + c5*s4*s6)), c1*(c2*(-c4*c6 + c5*s4*s6) - s2*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6)), c1*c2*(-c3*s5*s6 - s3*(c4*c5*s6 + c6*s4)) + s1*(-c3*(c4*c5*s6 + c6*s4) + s3*s5*s6), c1*(c2*c3*(c4*c6 - c5*s4*s6) + s2*(c4*c5*s6 + c6*s4)) - s1*s3*(c4*c6 - c5*s4*s6), c1*(c2*(-c3*c4*s5*s6 - c5*s3*s6) - s2*s4*s5*s6) + s1*(-c3*c5*s6 + c4*s3*s5*s6), c1*(c2*(c3*(c4*c5*c6 - s4*s6) - c6*s3*s5) + s2*(c4*s6 + c5*c6*s4)) + s1*(-c3*c6*s5 - s3*(c4*c5*c6 - s4*s6)), 0], [c1*(c2*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6) + s2*(-c4*c6 + c5*s4*s6)) - s1*(c3*s5*s6 + s3*(c4*c5*s6 + c6*s4)), s1*(c2*(-c4*c6 + c5*s4*s6) - s2*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6)), c1*(c3*(c4*c5*s6 + c6*s4) - s3*s5*s6) + c2*s1*(-c3*s5*s6 - s3*(c4*c5*s6 + c6*s4)), c1*s3*(c4*c6 - c5*s4*s6) + s1*(c2*c3*(c4*c6 - c5*s4*s6) + s2*(c4*c5*s6 + c6*s4)), c1*(c3*c5*s6 - c4*s3*s5*s6) + s1*(c2*(-c3*c4*s5*s6 - c5*s3*s6) - s2*s4*s5*s6), c1*(c3*c6*s5 + s3*(c4*c5*c6 - s4*s6)) + s1*(c2*(c3*(c4*c5*c6 - s4*s6) - c6*s3*s5) + s2*(c4*s6 + c5*c6*s4)), 0], [0, c2*(-c3*(c4*c5*s6 + c6*s4) + s3*s5*s6) - s2*(-c4*c6 + c5*s4*s6), s2*(c3*s5*s6 + s3*(c4*c5*s6 + c6*s4)), c2*(c4*c5*s6 + c6*s4) - c3*s2*(c4*c6 - c5*s4*s6), -c2*s4*s5*s6 + s2*(c3*c4*s5*s6 + c5*s3*s6), c2*(c4*s6 + c5*c6*s4) + s2*(-c3*(c4*c5*c6 - s4*s6) + c6*s3*s5), 0], [c1*(-c3*s5*(0.088*c6 + 0.107*s6) - s3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) - 0.0825*s3) - s1*(c2*(c3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.107*s6)) + s2*(c4*(-0.107*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.107*s6) - 0.0825*s4 + 0.316)), c1*(c2*(c4*(-0.107*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.107*s6) - 0.0825*s4 + 0.316) - s2*(c3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.107*s6))), c1*c2*(c3*s5*(-0.088*c6 - 0.107*s6) - s3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) - 0.0825*s3) + s1*(-c3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) - 0.0825*c3 + s3*s5*(0.088*c6 + 0.107*s6)), c1*(c2*c3*(c4*(0.107*c6 - 0.088*s6 - 0.384) - c5*s4*(0.088*c6 + 0.107*s6) + 0.0825*s4) + s2*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384))) - s1*s3*(c4*(0.107*c6 - 0.088*s6 - 0.384) - c5*s4*(0.088*c6 + 0.107*s6) + 0.0825*s4), c1*(c2*(-c3*c4*s5*(0.088*c6 + 0.107*s6) + c5*s3*(-0.088*c6 - 0.107*s6)) - s2*s4*s5*(0.088*c6 + 0.107*s6)) + s1*(-c3*c5*(0.088*c6 + 0.107*s6) + c4*s3*s5*(0.088*c6 + 0.107*s6)), c1*(c2*(c3*(c4*c5*(0.107*c6 - 0.088*s6) + s4*(-0.088*c6 - 0.107*s6)) + s3*s5*(-0.107*c6 + 0.088*s6)) + s2*(c4*(0.088*c6 + 0.107*s6) + c5*s4*(0.107*c6 - 0.088*s6))) + s1*(-c3*s5*(0.107*c6 - 0.088*s6) - s3*(c4*c5*(0.107*c6 - 0.088*s6) + s4*(-0.088*c6 - 0.107*s6))), 0], [c1*(c2*(c3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.107*s6)) + s2*(c4*(-0.107*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.107*s6) - 0.0825*s4 + 0.316)) - s1*(c3*s5*(0.088*c6 + 0.107*s6) + s3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*s3), s1*(c2*(c4*(-0.107*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.107*s6) - 0.0825*s4 + 0.316) - s2*(c3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.107*s6))), c1*(c3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*c3 - s3*s5*(0.088*c6 + 0.107*s6)) + c2*s1*(c3*s5*(-0.088*c6 - 0.107*s6) - s3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) - 0.0825*s3), c1*s3*(c4*(0.107*c6 - 0.088*s6 - 0.384) - c5*s4*(0.088*c6 + 0.107*s6) + 0.0825*s4) + s1*(c2*c3*(c4*(0.107*c6 - 0.088*s6 - 0.384) - c5*s4*(0.088*c6 + 0.107*s6) + 0.0825*s4) + s2*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384))), c1*(c3*c5*(0.088*c6 + 0.107*s6) - c4*s3*s5*(0.088*c6 + 0.107*s6)) + s1*(c2*(-c3*c4*s5*(0.088*c6 + 0.107*s6) + c5*s3*(-0.088*c6 - 0.107*s6)) - s2*s4*s5*(0.088*c6 + 0.107*s6)), c1*(c3*s5*(0.107*c6 - 0.088*s6) + s3*(c4*c5*(0.107*c6 - 0.088*s6) + s4*(-0.088*c6 - 0.107*s6))) + s1*(c2*(c3*(c4*c5*(0.107*c6 - 0.088*s6) + s4*(-0.088*c6 - 0.107*s6)) + s3*s5*(-0.107*c6 + 0.088*s6)) + s2*(c4*(0.088*c6 + 0.107*s6) + c5*s4*(0.107*c6 - 0.088*s6))), 0], [0, c2*(-c3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) - 0.0825*c3 + s3*s5*(0.088*c6 + 0.107*s6)) - s2*(c4*(-0.107*c6 + 0.088*s6 + 0.384) + c5*s4*(0.088*c6 + 0.107*s6) - 0.0825*s4 + 0.316), s2*(-c3*s5*(-0.088*c6 - 0.107*s6) + s3*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) + 0.0825*s3), c2*(c4*c5*(0.088*c6 + 0.107*s6) - 0.0825*c4 - s4*(-0.107*c6 + 0.088*s6 + 0.384)) - c3*s2*(c4*(0.107*c6 - 0.088*s6 - 0.384) - c5*s4*(0.088*c6 + 0.107*s6) + 0.0825*s4), -c2*s4*s5*(0.088*c6 + 0.107*s6) + s2*(c3*c4*s5*(0.088*c6 + 0.107*s6) - c5*s3*(-0.088*c6 - 0.107*s6)), c2*(c4*(0.088*c6 + 0.107*s6) + c5*s4*(0.107*c6 - 0.088*s6)) + s2*(-c3*(c4*c5*(0.107*c6 - 0.088*s6) + s4*(-0.088*c6 - 0.107*s6)) - s3*s5*(-0.107*c6 + 0.088*s6)), 0]])))(_0[0],_0[1],_0[2],_0[3],_0[4],_0[5],_0[6],_1[0],_1[1],_1[2],_1[3],_1[4],_1[5],_1[6])
