'''
Test script that moves the arm in discrete steps and plots
'''

import time

import matplotlib.pyplot as plt
from pyrobot import Robot

from DMP import plot_trajectory

robot = Robot('locobot')

# Move to start position. Adjust height for diff tool.
robot.arm.set_ee_pose_pitch_roll([0.418, 0., 0.15], pitch=1.57, roll=0, plan=True)
time.sleep(2)

# Lists to save values for plotting.
ee_pos = []
Time=[]

#position=np.array([0.418, 0.0, 0.15])
#position1=np.array([-0.20, 0.0, 0.0])
#orientation=np.array([[0.0,0.0,1.0],[0.0,1.0,0.0],[-1,0.0,0.0]])

# start position
#robot.arm.set_ee_pose_pitch_roll([0.418, 0., 0.20], pitch=1.57, roll=0, plan=True)
#time.sleep(2)
# robot.arm.set_ee_pose(position, orientation, plan=True, wait=True)
# robot.arm.move_ee_xyz(position1, plan=False, numerical=False)
# time.sleep(2)

#ee_pos = []
#Time=[]
#start = time.time()
#snooze = time.time()
# position2=np.array([0.0, 0.15, 0.0])
# time.sleep(2)
# robot.arm.move_ee_xyz(position2, plan=False, numerical=False)


config = dict(moveit_planner='ESTkConfigDefault')
robot = Robot('locobot', arm_config=config)
target_joints = [[0.408, 0.721, -0.471, -1.4, 0.920],
				[0.388, 0.721, -0.471, -1.4, 0.920],
				[0.368, 0.721, -0.471, -1.4, 0.920]	
				]

robot.arm.go_home()

for joint in target_joints:
	robot.arm.set_joint_positions(joint, plan=True)
	time.sleep(2)

robot.arm.go_home()	






####for i in range(7):
	# position2 = np.array([-0.02, 0, 0])
	# robot.arm.move_ee_xyz(position2, plan=False, numerical=False)

	# robot.arm.set_ee_pose_pitch_roll([0.418 - 0.02 * i, 0.0, 0.15], pitch=1.57, roll=0, plan=False, wait=False)
	###position=np.array([0.418-0.02*i, 0.0, 0.15])
	###orientation=np.array([[0.0,0.0,1.0],[0.0,1.0,0.0],[-1,0.0,0.0]])
	###robot.arm.set_ee_pose(position, orientation, plan=False, wait=False)
	# robot.arm.move_ee_xyz(position, plan=False, numerical=False)
	###while time.time() - snooze < 2:
		###elaptime = time.time() - start
		###pos = robot.arm.pose_ee[0]
		###Time.append(elaptime)
		###ee_pos.append(pos)
	###snooze = time.time()
###fig,axes = plt.subplots(3)

###axes[0].plot(Time, [ee_pos[i][0] for i in range(len(ee_pos))])
###axes[1].plot(Time, [ee_pos[i][1] for i in range(len(ee_pos))])
###axes[2].plot(Time, [ee_pos[i][2] for i in range(len(ee_pos))])
###print(ee_pos[0], ee_pos[-1])

###plt.show()


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
plot_trajectory(ee_pos)

print(f'First pos: {ee_pos[0]}\n Last pos:{ee_pos[-1]}')


'''
sample output from pose_ee. xyz, rotation matrix, quarternions
(array([[0.40872469],
       [0.00392196],
       [0.21296304]]), array([[-7.35591238e-02,  1.49657630e-03,  9.97289735e-01],
       [-9.02751605e-04,  9.99998364e-01, -1.56722707e-03],
       [-9.97290449e-01, -1.01558876e-03, -7.35576525e-02]]), array([ 2.02628528e-04,  7.32651885e-01, -8.81324363e-04,  6.80602966e-01]))
'''

