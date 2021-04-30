

from pyrobot import Robot
import time
import numpy as np
import matplotlib.pyplot as plt

arm_config=dict(control_mode = 'torque')
robot = Robot('locobot', arm_config=arm_config)



#robot.arm.go_home()

computed_position_angles = [0.03325869461586744, -1.4725916096806952, 1.5243097190194765, 0.2415486924243831, -3.145636303331413]


robot.arm.set_joint_positions(computed_position_angles, plan=False, wait=False)
