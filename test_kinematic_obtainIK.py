
### 1)obtain the HGOAL(final position) through kinematic training 
### 2)use the final pose from kinematic training as HGOAL
### 3)Run set_joint_positions  



from pyrobot import Robot
import Locobot
import time
import numpy as np
import matplotlib.pyplot as plt

qInit = [-0.02761165, -1.28394198, 1.65823328, 0.49087387 ,-0.0] ### inital Joint positions

mybot = Locobot.Locobot()

arm_config=dict(control_mode = 'torque')
robot = Robot('locobot', arm_config=arm_config)

# Lists to save values for plotting.
ee_pos = []
current_pose = []
current_orientation = []
current_quat = []
Time=[]
Final_Goal = []


target_torque=4*[0]
robot.arm.set_joint_torques(target_torque)

print("Go to Start position")
time.sleep(2)
print("3..")
time.sleep(1)
print("2..")
time.sleep(1)
print("1..")
time.sleep(1)
print("Starting to record")


# Parameters
STEPS = 1
TIME_BETWEEN_STEPS = 3

start = time.time() # Time since start, for plotting
lap = time.time() # Time since last step, for checking loop condition
for i in range(STEPS):
    # Save end effector positions.
	while time.time() - lap < TIME_BETWEEN_STEPS:
		elaptime = time.time() - start
		ee_pose = robot.arm.get_ee_pose(robot.arm.configs.ARM.ARM_BASE_FRAME)
		cur_pos, cur_ori, cur_quat = ee_pose
		current_pose.append(cur_pos)
		current_orientation.append(cur_ori)
		current_quat.append(cur_quat)
		Time.append(elaptime)
	lap = time.time()
last_joint_angles = robot.arm.get_joint_angles()
print('Recording stopped')


##import pdb; pdb.set_trace()
total_positions = len(current_pose)
Final_Goal = current_pose[total_positions-1]
print(total_positions)
HGoal = np.array([[1,0,0,Final_Goal[0].item()],[0,1,0,Final_Goal[1].item()],[0,0,1,Final_Goal[2].item()],[0,0,0,1]])

###print(current_pose)
###print(last_joint_angles)

q,Err=mybot.IterInvKin(qInit, HGoal)
print('error', Err)
print('from get_joint_angles', last_joint_angles)
print('compute IK angles', q)
difference_angles = last_joint_angles - q
print('difference in angles', difference_angles )

#robot.arm.set_joint_positions(q, plan=True, wait=True)
