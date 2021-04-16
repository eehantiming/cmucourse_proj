'''
Replays the recorded trajectory by end effector position.
roslaunch locobot_control main.launch use_arm:=true torque_control:=false use_rviz:=false
python3 replay_traj.py
'''

import time
import numpy as np
import matplotlib.pyplot as plt


# from pyrobot import Robot

# robot = Robot('locobot')

# Change file to load here.
filename = 'straight.npz'

times = np.load(filename)['time']
positions = np.load(filename)['pos']
velocities = np.load(filename)['vel']
accelerations = np.load(filename)['acc']
print(f'Number of points: {len(positions)}')
# print(f'lowest:{min(positions)}')
# import pdb;pdb.set_trace()
fig,axes = plt.subplots(3)
axes[0].plot([positions[i][0] for i in range(len(positions))])
axes[1].plot([positions[i][1] for i in range(len(positions))])
axes[2].plot([positions[i][2] for i in range(len(positions))])
plt.show()
# for position in positions:
    # position = position.tolist()
    # robot.arm.set_ee_pose_pitch_roll(position, pitch=1.57, roll=0, plan=False)
    # time.sleep(0.1)
    # print(position)



