from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *
import numpy as np
from math import pi
import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning) 

dh_d = [0.25, 0.15, 0.2, 0, 0, 0.1]
dh_a = [0, 0, 0, 0, 0, 0]
dh_alpha = [-1.5708, 1.5708, 0, -1.5708, 1.5708, 0]
dh_theta = [0, 0, 0, 0, 0, 0]
dh_params = np.stack((dh_d,dh_a,dh_alpha,dh_theta), axis=-1)
robot = RobotSerial(dh_params)

abc1 = np.array([[-0.266], [0.031], [0.304]])
abc2 = np.array([[-0.232], [0.192], [0.357]])
abc3 = np.array([[-0.076], [0.225], [0.461]])
abc4 = np.array([[-0.014], [0.262], [0.262]])
abc5 = np.array([[-0.221], [0.020], [0.482]])
abc6 = np.array([[-0.266], [0.031], [0.304]])

xyz1 =  np.array([-0.665994, 0.308907, 0.665997])
xyz2 =  np.array([-0.71637, 0.398236, 0.71637])
xyz3 =  np.array([0.0292315, 0.541703, 0.91522])
xyz4 =  np.array([0.158575, 0.957389, 0.50314])
xyz5 =  np.array([-0.35985, 0.225772, 0.924447])
xyz6 =  np.array([-0.666147, 0.308929, 0.666111])

frame1 = Frame.from_euler_3(xyz1, abc1)
frame2 = Frame.from_euler_3(xyz2, abc2)
frame3 = Frame.from_euler_3(xyz3, abc3)
frame4 = Frame.from_euler_3(xyz4, abc4)
frame5 = Frame.from_euler_3(xyz5, abc5)
frame6 = Frame.from_euler_3(xyz6, abc6)

frames = [frame1, frame2, frame3, frame4, frame5, frame6]

trajectory = RobotTrajectory(robot, frames)
trajectory.show(motion='lin')
