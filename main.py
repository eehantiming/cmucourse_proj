'''
Break down input image to points. Execute strokes with learned DMP.

roslaunch locobot_control main.launch use_arm:=true torque_control:=false use_rviz:=false
python3 main.py 
'''

import numpy as np
from pyrobot import Robot

from CV.get_vertices import get_vertices
from DMP import DMP

DRAWING_HEIGHT = 0.26

# Get vertices with CV module
image = './CV/house3.jpg'
points = get_vertices(image)
# Append first point as ending point for last stroke
points.append(points[0][:])

# Initialize DMP and learn weights
print('Learning weights')
DMP_straight = DMP(5)
DMP_straight.learn_weights(f'./recordings/forward_filtered.npz')

# Initialize robot
robot = Robot('locobot')
print('Moving')
robot.arm.go_home()

# Generate trajectories and draw each stroke
print('Generating trajectories')
start = points[0]
start.append(DRAWING_HEIGHT)
for i in range (1, len(points)):
    end = points[i]
    end.append(DRAWING_HEIGHT)
    print('\tDrawing from {start} to {end}..')
    # start at a higher point
    pre_pos = start[:]
    pre_pos[2] = DRAWING_HEIGHT + 0.1
    robot.arm.set_ee_pose_pitch_roll(pre_pos, pitch=1.57, roll=0, plan=False, numerical=False)

    trajectory = DMP_straight.generate_traj(start, end)

    previous = [0.,0.,0.] 
    for position in trajectory:
        # Move if displacement is significant
        disp = np.linalg.norm(np.array(position) - np.array(previous))
        if disp > 2.5e-03:
            robot.arm.set_ee_pose_pitch_roll(position, pitch=1.57, roll=0, plan=False, numerical=False)
            previous = position
        else:
            print(f'Smol disp {disp}.')
    # Use current point as the start for next stroke
    start = end
    # Lift the arm before going to home
    robot.arm.set_ee_pose_pitch_roll(pre_pos, pitch=1.57, roll=0, plan=False, numerical=False)
    robot.arm.go_home()

print('Drawing Done!')