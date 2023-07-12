#Inverse Kinematic Controller for Hexapod Gyration 

#in this case, O vector and euler angles are input (pose)
#output is 18 angles, one for each motor for given pose  

#additional required information is end effector position for each leg and hip positions vals WRT body  

import numpy as np
from array import *
from hexapod import *
####robot parameters---------------------------------------------------------------------------------------
body_radius = Hexapod.body_radius
leg_spread = Hexapod.leg_spread
seg1 = Hexapod.seg1
seg2 = Hexapod.seg2
seg3 = Hexapod.seg3

#position of hip joints WRT body 
s_1  = np.array([[body_radius*np.cos(leg_spread/2)],
                 [body_radius*np.sin(leg_spread/2)],
                 [0]])

s_2 = np.array([[body_radius*np.cos(leg_spread/2)],
                [-body_radius*np.sin(leg_spread/2)],
                [0]])

s_3  = np.array([[body_radius*np.sin((np.pi/6) - (leg_spread/2))],
                 [body_radius*np.cos((np.pi/6) - (leg_spread/2))],
                 [0]])

s_4  = np.array([[body_radius*np.cos((np.pi/3) + (leg_spread/2))],
                 [-body_radius*np.sin((np.pi/3) + (leg_spread/2))],
                 [0]])

s_5  = np.array([[-body_radius*np.sin((np.pi/6) + (leg_spread/2))],
                 [body_radius*np.cos((np.pi/6) + (leg_spread/2))],
                 [0]])

s_6  = np.array([[-body_radius*np.cos((np.pi/3) - (leg_spread/2))],
                 [-body_radius*np.sin((np.pi/3) - (leg_spread/2))],
                 [0]])

s_i = [s_1, s_2, s_3, s_4, s_5, s_6]

#position of end effectors 
#outward extension length of leg WRT horizontal (mm), needs to be found using FK 
ee_ext = 358 + body_radius 

u_1 = np.array([[ee_ext*np.cos(np.pi/6)],
                [ee_ext*np.sin(np.pi/6)],
                [0]])

u_2 = np.array([[ee_ext*np.cos(np.pi/6)],
                [-ee_ext*np.sin(np.pi/6)],
                [0]])

u_3 = np.array([[0],
                [ee_ext],
                [0]])

u_4 = np.array([[0],
                [-ee_ext],
                [0]])

u_5 = np.array([[-ee_ext*np.cos(np.pi/6)],
                [ee_ext*np.sin(np.pi/6)],
                [0]])

u_6 = np.array([[-ee_ext*np.cos(np.pi/6)],
                [-ee_ext*np.sin(np.pi/6)],
                [0]])

u_i = [u_1, u_2, u_3, u_4, u_5, u_6]

####INPUTS--------------------------------------------------------------------------------------------------
O_old = np.array([[5],
              [0],
              [113.18]]) #mm

#avoid z-axis rotation as it fucks up legs 3 and 4 due to directional ambiguity 
##maybe patch in future by checking direction of rotation to adjust alpha motor modifier
euler_angles_deg_old = np.array([[5],
                             [5],
                             [0]])

def gyration(O, euler_angles_deg):
    #convert degrees to radians 
    euler_angles_rad = euler_angles_deg * np.pi/180

    a = euler_angles_rad[0][0]
    b = euler_angles_rad[1][0]
    c = euler_angles_rad[2][0]

    ####first loop closure-------------------------------------------------------------------------------------
    ####find l_i values to calculate alpha (hip) value for each leg 

    #R transformation matrix 
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(a), -np.sin(a)],
                   [0, np.sin(a), np.cos(a)]])

    Ry = np.array([[np.cos(b), 0, np.sin(b)],
                   [0, 1, 0],
                   [-np.sin(b), 0, np.cos(b)]])

    Rz = np.array([[np.cos(c), -np.sin(c), 0],
                   [np.sin(c), np.cos(c), 0],
                   [0, 0, 1]])

    #needs to be calculated sequentially as ORDER MATTERS
    R = np.array(np.dot(np.dot(Rx, Ry), Rz))

    #calculate R_si values for each hip joint 
    Rs_1 = np.array(np.dot(R, s_1))
    Rs_2 = np.array(np.dot(R, s_2))
    Rs_3 = np.array(np.dot(R, s_3))
    Rs_4 = np.array(np.dot(R, s_4))
    Rs_5 = np.array(np.dot(R, s_5))
    Rs_6 = np.array(np.dot(R, s_6))

    Rs_i = [Rs_1, Rs_2, Rs_3, Rs_4, Rs_5, Rs_6]

    #calculate l_i values for each leg
    l_i = O + Rs_i - u_i

    #calculate alpha angle for each leg in degrees (WRT to ground bird's eye view)
    a_i = np.array([np.arctan(l_i[0][1]/l_i[0][0]),
                    np.arctan(l_i[1][1]/l_i[1][0]),
                    np.arctan(l_i[2][1]/l_i[2][0]),
                    np.arctan(l_i[3][1]/l_i[3][0]),
                    np.arctan(l_i[4][1]/l_i[4][0]),
                    np.arctan(l_i[5][1]/l_i[5][0])])  * 180/np.pi

    a_i_mod = np.array([[-30],
                        [30],
                        [90],
                        [-90],
                        [30],
                        [-30]])

    #adjust alpha values WRT to each hip motor 
    #look up matrix addition 
    alpha = np.array([a_i[0][0] + a_i_mod[0][0],
                      a_i[1][0] + a_i_mod[1][0],
                      a_i[2][0] + a_i_mod[2][0],
                      a_i[3][0] + a_i_mod[3][0],
                      a_i[4][0] + a_i_mod[4][0],
                      a_i[5][0] + a_i_mod[5][0]])

    ####second loop closure----------------------------------------------------------------------------------------------
    ####find s_i2 values 

    a_irad = np.array(np.dot(a_i, np.pi/180))

    s_12 = np.array([s_i[0][0] + seg1*np.cos(a_irad[0][0]),
                     s_i[0][1] + seg1*np.sin(a_irad[0][0]),
                     s_i[0][2]])

    s_22 = np.array([s_i[1][0] + seg1*np.cos(a_irad[1][0]),
                     s_i[1][1] + seg1*np.sin(a_irad[1][0]),
                     s_i[1][2]])

    s_32 = np.array([s_i[2][0] - seg1*np.cos(a_irad[2][0]),
                     s_i[2][1] - seg1*np.sin(a_irad[2][0]),
                     s_i[2][2]])

    s_42 = np.array([s_i[3][0] - seg1*np.cos(a_irad[3][0]),
                     s_i[3][1] - seg1*np.sin(a_irad[3][0]),
                     s_i[3][2]])

    s_52 = np.array([s_i[4][0] - seg1*np.cos(a_irad[4][0]),
                     s_i[4][1] - seg1*np.sin(a_irad[4][0]),
                     s_i[4][2]])

    s_62 = np.array([s_i[5][0] - seg1*np.cos(a_irad[5][0]),
                     s_i[5][1] - seg1*np.sin(a_irad[5][0]),
                     s_i[5][2]])

    s_i2 = [s_12, s_22, s_32, s_42, s_52, s_62]

    ####third loop closure--------------------------------------------------------------------------------------------------
    ####find l_i2 values 

    Rs_i2 = np.matmul(R, s_i2)

    l_i2 = O + Rs_i2 - u_i

    ####fourth loop closure------------------------------------------------------------------------------------------------
    ####calculate beta and gamma values 

    #take absolute value of l_i2 values 
    ##test = [number**2 for number in l_i2[0,:]]
    l_i2abs = np.array([[np.sqrt(np.array(list(map(lambda x: x**2, l_i2[0,:]))).sum())],
                        [np.sqrt(np.array(list(map(lambda x: x**2, l_i2[1,:]))).sum())],
                        [np.sqrt(np.array(list(map(lambda x: x**2, l_i2[2,:]))).sum())],
                        [np.sqrt(np.array(list(map(lambda x: x**2, l_i2[3,:]))).sum())],
                        [np.sqrt(np.array(list(map(lambda x: x**2, l_i2[4,:]))).sum())],
                        [np.sqrt(np.array(list(map(lambda x: x**2, l_i2[5,:]))).sum())]])

    rho = np.array([np.arctan(l_i2[0][2]/np.sqrt(l_i2[0][0]**2 + l_i2[0][1]**2)),
                      np.arctan(l_i2[1][2]/np.sqrt(l_i2[1][0]**2 + l_i2[1][1]**2)),
                      np.arctan(l_i2[2][2]/np.sqrt(l_i2[2][0]**2 + l_i2[2][1]**2)),
                      np.arctan(l_i2[3][2]/np.sqrt(l_i2[3][0]**2 + l_i2[3][1]**2)),
                      np.arctan(l_i2[4][2]/np.sqrt(l_i2[4][0]**2 + l_i2[4][1]**2)),
                      np.arctan(l_i2[5][2]/np.sqrt(l_i2[5][0]**2 + l_i2[5][1]**2))])

    phi = np.array([np.arcsin((l_i2[0][2] - l_i[0][2])/seg1),
                    np.arcsin((l_i2[1][2] - l_i[1][2])/seg1),
                    np.arcsin((l_i2[2][2] - l_i[2][2])/seg1),
                    np.arcsin((l_i2[3][2] - l_i[3][2])/seg1),
                    np.arcsin((l_i2[4][2] - l_i[4][2])/seg1),
                    np.arcsin((l_i2[5][2] - l_i[5][2])/seg1)])

    beta = np.array([[np.arccos((seg2**2 + l_i2abs[0][0]**2 - seg3**2)/(2 * seg2 * l_i2abs[0][0])) - (rho[0][0] + phi[0][0])],
                     [np.arccos((seg2**2 + l_i2abs[1][0]**2 - seg3**2)/(2 * seg2 * l_i2abs[1][0])) - (rho[1][0] + phi[1][0])],
                     [np.arccos((seg2**2 + l_i2abs[2][0]**2 - seg3**2)/(2 * seg2 * l_i2abs[2][0])) - (rho[2][0] + phi[2][0])],
                     [np.arccos((seg2**2 + l_i2abs[3][0]**2 - seg3**2)/(2 * seg2 * l_i2abs[3][0])) - (rho[3][0] + phi[3][0])],
                     [np.arccos((seg2**2 + l_i2abs[4][0]**2 - seg3**2)/(2 * seg2 * l_i2abs[4][0])) - (rho[4][0] + phi[4][0])],
                     [np.arccos((seg2**2 + l_i2abs[5][0]**2 - seg3**2)/(2 * seg2 * l_i2abs[5][0])) - (rho[5][0] + phi[5][0])]]) * 180/np.pi

    gamma = np.array([[np.pi - np.arccos((seg2**2 + seg3**2 - l_i2abs[0][0]**2)/(2*seg2*seg3))],
                      [np.pi - np.arccos((seg2**2 + seg3**2 - l_i2abs[1][0]**2)/(2*seg2*seg3))],
                      [np.pi - np.arccos((seg2**2 + seg3**2 - l_i2abs[2][0]**2)/(2*seg2*seg3))],
                      [np.pi - np.arccos((seg2**2 + seg3**2 - l_i2abs[3][0]**2)/(2*seg2*seg3))],
                      [np.pi - np.arccos((seg2**2 + seg3**2 - l_i2abs[4][0]**2)/(2*seg2*seg3))],
                      [np.pi - np.arccos((seg2**2 + seg3**2 - l_i2abs[5][0]**2)/(2*seg2*seg3))]]) * 180/np.pi
    return [alpha,beta,gamma]

