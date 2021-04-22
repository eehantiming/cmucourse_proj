'''
Execute motion, saves and plots xyz.
'''

import time

import matplotlib.pyplot as plt
from pyrobot import Robot

robot = Robot('locobot')

# Move to start position. Adjust height for diff tool.
robot.arm.set_ee_pose_pitch_roll([0.418, 0., 0.15], pitch=1.57, roll=0, plan=True)
time.sleep(2)

# Lists to save values for plotting.
ee_pos = []
Time=[]

# Parameters
STEPS = 10
TIME_BETWEEN_STEPS = 2

start = time.time() # Time since start, for plotting
lap = time.time() # Time since last step, for checking loop condition
for i in range(STEPS):
    # Move to next point. use wait=False so that values are saved while moving.
    robot.arm.set_ee_pose_pitch_roll([0.418 - 0.02 * i, 0.0, 0.15], pitch=1.57, roll=0, plan=False, wait=False)
    # Save end effector positions.
    while time.time() - lap < TIME_BETWEEN_STEPS:
        elaptime = time.time() - start
        pos = robot.arm.pose_ee[0]
        Time.append(elaptime)
        ee_pos.append(pos)
    lap = time.time()

# Plot EE pose over time.
y_labels = ['X', 'Y', 'Z']
fig,axes = plt.subplots(3)
for axis in range(3): # x y z
    axes[axis].plot(Time, [ee_pos[i][axis] for i in range(len(ee_pos))])
    axes[axis].set_xlabel('Time (s)')
    axes[axis].set_ylabel(f'{y_labels[axis]} (m)')
    axes[axis].annotate(str(ee_pos[-1][axis]), xy=(time[-1], ee_pos[-1][axis]))

plt.show()

print(f'First pos: {ee_pos[0]}\n Last pos:{ee_pos[-1]}')


'''
sample output from pose_ee. xyz, rotation matrix, quarternions
(array([[0.40872469],
       [0.00392196],
       [0.21296304]]), array([[-7.35591238e-02,  1.49657630e-03,  9.97289735e-01],
       [-9.02751605e-04,  9.99998364e-01, -1.56722707e-03],
       [-9.97290449e-01, -1.01558876e-03, -7.35576525e-02]]), array([ 2.02628528e-04,  7.32651885e-01, -8.81324363e-04,  6.80602966e-01]))
'''

