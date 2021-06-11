'''
Records a human controlled trajectory for a skill. Saves it for fitting of a DMP.

roslaunch locobot_control main.launch use_arm:=true torque_control:=true use_rviz:=false
load_pyrobot_env && python3 record_traj.py
'''

import time
import numpy as np
from scipy.signal import savgol_filter

from pyrobot import Robot

# Time in seconds to record for
DURATION = 15
TIME_BETWEEN_STEPS = 0.01

arm_config = dict(control_mode='torque')
robot = Robot('locobot', arm_config=arm_config)

# Set torques to 0 for human control
# TODO: need to move arm to above paper first?
target_torque = 4 * [0]
robot.arm.set_joint_torques(target_torque)

# Main loop to record trajectory
# TODO: need to save rpy?
pos_array = []
vel_array = []
acc_array = []
time_array = []
elaptime = 0

print("Go to start position")
for t in range(3,0,-1):
    print(f'\r{t}...', end='')
    time.sleep(1)

print("\nStarting to record")

# Save first position and velocity
old_pos = np.concatenate((robot.arm.pose_ee[0]), axis=None)
# old_vel = np.zeros_like(old_pos)
# acc = np.zeros_like(old_pos)
pos_array.append(old_pos)
# vel_array.append(old_vel)
# acc_array.append(acc)

start = time.time()
while elaptime < DURATION:
    elaptime = time.time() - start
    # TODO: Check data format
    pos = np.concatenate((robot.arm.pose_ee[0]), axis=None)
    # use previous values to estimate differential
    # vel = (pos - old_pos) / TIME_BETWEEN_STEPS
    # acc = (vel - old_vel) / TIME_BETWEEN_STEPS
    time_array.append(elaptime)
    pos_array.append(pos)
    # vel_array.append(vel)
    # acc_array.append(acc)
    # old_vel = vel
    # old_pos = pos
    time.sleep(TIME_BETWEEN_STEPS)

pos_array = np.array(pos_array)
vel_array = (pos_array[1:, :] - pos_array[:-1, :]) / TIME_BETWEEN_STEPS
for i in range(vel_array.shape[1]): 
	vel_array[:, i] = savgol_filter(vel_array[:, i], 21, 5)
vel_array = np.vstack([vel_array, vel_array[-1:, :]]) 
acc_array = (vel_array[1:, :] - vel_array[:-1, :]) / TIME_BETWEEN_STEPS
acc_array = np.vstack([acc_array, acc_array[-1:, :]])

# Save lists. auto saves to .npz 
filename = './recordings/' + input('Save as: ')
np.savez(filename, time=time_array, pos=pos_array, vel=vel_array, acc=acc_array)
print(f'Saved to {filename}.npz!')

# TODO: add plots for pos, vel and acc
# vel = (x[1:, :] - x[:-1, :]) / dt
# for i in range(vel.shape[1]): 
# 	vel[:, i] = savgol_filter(vel[:, i], 21, 5)
# 	vel = np.vstack([vel, vel[-1:, :]]) 
# 	acc = (vel[1:, :] - vel[:-1, :]) / dt
# 	acc = np.vstack([acc, acc[-1:, :]])
