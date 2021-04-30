import numpy as np

from pyrobot import Robot

# from CV.get_vertices import get_vertices
from DMP import DMP

DRAWING_HEIGHT = 0.26

# Get vertices with CV module
image = './CV/house.jpg'
# points = get_vertices(image) # [[s1,e1],[s2,e2],[s3,e3],...]
points = [ [[0.3,0.2], [0.35,0]], [[0.35,0], [0.3,-0.2]], [[0.3,-0.2],[0.3,0.2]] ]

# Initialize DMP and learn weights
print('Learning weights')
DMP_straight = DMP(5)
DMP_straight.learn_weights(f'./recordings/forward.npz')

# Initialize robot
robot = Robot('locobot')
print('Moving')
robot.arm.go_home()

# Generate trajectory and draw each stroke
print('Generating trajectories')
for start, end in points:
	print(start,end)
	# import pdb; pdb.set_trace()
	start.append(DRAWING_HEIGHT)
	end.append(DRAWING_HEIGHT)
	print(start,end)

	trajectory = DMP_straight.generate_traj(start, end)

	previous = [0.,0.,0.]
	# TODO: Move out first to prevent hitting robot.
	for position in trajectory:
	    # Move if displacement is significant
	    disp = np.linalg.norm(np.array(position) - np.array(previous))
	    if disp > 2.5e-03:
	        robot.arm.set_ee_pose_pitch_roll(position, pitch=1.57, roll=0, plan=False, numerical=False)
	        previous = position
	    else:
	        print(f'Smol disp {disp}.')

print('Drawing Done!')
robot.arm.move_ee_xyz(np.array((0,0,0.05)))