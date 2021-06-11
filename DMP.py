'''
Fits a DMP from npz recording of EE xyz position velocity and acceleration. 
Generates trajectory for the learned skill.

roslaunch locobot_control main.launch use_arm:=true torque_control:=false use_rviz:=false
python3 DMP.py forward_filtered
'''

import math
import sys
import numpy as np
import matplotlib.pyplot as plt

from pyrobot import Robot


class DMP:

    def __init__(self, num_basis):
        self.K = 25 * 25 / 4
        self.B = 25
        self.centers = np.linspace(0, 1, num_basis) # Basis function centers
        self.width = (0.65 * (1. / (num_basis - 1.)) ** 2) # Basis function widths
        self.duration = 3 # TODO: Load this from recording
        self.dt = 0.1

    def learn_weights(self, filename):
        '''
        Learns and saves DMP weights from recordings
        '''
        print(f'Loading from {filename}!')
        data = np.load(filename)
        times = data['time']
        positions = data['pos']
        velocities = data['vel']
        accelerations = data['acc']
        duration = 3.0 # TODO: Load this from recording

        PHI = []
        forcing_function = []
        for i, time_ in enumerate(times):
            phi = [math.exp(-0.5 * ((time_ / duration) - center) ** 2 / self.width) for center in self.centers]
            phi = phi / np.sum(phi)
            PHI.append(phi)

            f = ((accelerations[i] * duration ** 2) - self.K * (positions[-1] - positions[i]) + self.B * (velocities[i] * duration)) / (positions[-1] - positions[0])
            forcing_function.append(f)

        # Calculate weights via linear regression
        self.weights = np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(PHI),PHI)),np.transpose(PHI)), forcing_function)

        ## Save the weights file to save relearning time
        # np.save('DMPweights.npy', self.weights)
        # print('Saved weights to DMPweights.npy!')

    def load_weights(self, filename):
        '''
        Load weights from npy file
        '''
        print(f'Loading weights from {filename}!')
        self.weights = np.load(filename)

    def generate_traj(self, start, goal):
        '''
        From weights, generate a trajectory from start to goal.
        start, goal: python list of floats xyz positions
        Returns: list of lists of xyz positions
        '''
        pose_start = np.array(start)
        pose = np.copy(pose_start)
        pose_goal = np.array(goal)
        vel = np.zeros(3)
        acc = np.zeros(3)
        dmp_trajectory = []
        dmp_trajectory.append(pose_start.tolist())

        t = 0
        for i in range(50):
            t = t + self.dt
            if t < self.duration:
                Phi = [math.exp(-0.5 * ((t/self.duration) - center) ** 2 / self.width) for center in self.centers]
                Phi = Phi / np.sum(Phi)
                force = np.dot(Phi, self.weights)
            else: 
                force = 0
            acc = self.K * (pose_goal - pose) / (self.duration ** 2) - self.B * vel / self.duration + (pose_goal - pose_start) * force / (self.duration ** 2)
            vel = vel + acc * self.dt
            # import pdb;pdb.set_trace()
            pose = pose + vel * self.dt
            dmp_trajectory.append(pose.tolist())

        return dmp_trajectory


def plot_trajectory(trajectory):
    ''' 
    Helper function used to create and show subplots in xyz
    input: lists of lists [[x,y,z], [x,y,z], ...]
    '''
    fig, axes = plt.subplots(3)
    labels = ['X', 'Y', 'Z']
    for j in range(3):
        axes[j].plot([trajectory[i][j] for i in range(len(trajectory))])
        # Label first and last point
        axes[j].annotate(str(trajectory[0][j]),xy=(0, trajectory[0][j]))
        axes[j].annotate(str(trajectory[-1][j]),xy=(len(trajectory), trajectory[-1][j]))
        axes[j].set_ylabel(f'{labels[j]}(m)')
    plt.show()

if __name__ == "__main__":
    # Create a DMP object
    DMP = DMP(5)

    # Learn or load weights
    print('Learning weights')
    DMP.learn_weights(f'./recordings/{sys.argv[1]}.npz')
    # DMP.load_weights('DMPweights.npy')
    
    # Generate trajectory from start to goal
    print('Generating trajectory')
    #vertical
    trajectory = DMP.generate_traj([0.25, 0., 0.26], [0.35, 0., 0.26])
    # #hori
    # trajectory2 = DMP.generate_traj([0.25, 0., 0.26], [0.25, 0.1, 0.26])
    # #slanted
    # trajectory3 = DMP.generate_traj([0.25, 0., 0.26], [0.35, 0.1, 0.26])
    # offset
    # trajectory4 = DMP.generate_traj([0.25, 0.1, 0.26], [0.35, 0.1, 0.26])
    # trajectory5 = DMP.generate_traj([0.25, -0.1, 0.26], [0.35, -0.1, 0.26])

    plot_trajectory(trajectory)
    # plot_trajectory(trajectory2)
    # plot_trajectory(trajectory3)
    # plot_trajectory(trajectory4)
    # plot_trajectory(trajectory5)
    # print(trajectory)

    # Create robot and move arm according to trajectory
    robot = Robot('locobot')
    print('Moving')
    # Move to start position without touching paper
    robot.arm.go_home()
    robot.arm.set_ee_pose_pitch_roll([0.25, 0., 0.35], pitch=1.57,roll=0, plan=False,numerical=False)

    previous = [0.,0.,0.]
    for position in trajectory:
        # Move if displacement is significant
        disp = np.linalg.norm(np.array(position) - np.array(previous))
        if disp > 2.5e-03:
            robot.arm.set_ee_pose_pitch_roll(position, pitch=1.57, roll=0, plan=False, numerical=False)
            previous = position
        else:
            print(f'Smol disp {disp}.')
    robot.arm.go_home()
    
    # # Traj 2 and 3
    # robot.arm.set_ee_pose_pitch_roll([0.25, 0., 0.3], pitch=1.57,roll=0, plan=False,numerical=False)
    # # TODO: set position directly in generate_traj?
    # previous = [0.,0.,0.]
    # # TODO: Move out first to prevent hitting robot.
    # for position in trajectory2:
    #     # Move if displacement is significant
    #     disp = np.linalg.norm(np.array(position) - np.array(previous))
    #     if disp > 2.5e-03:
    #         robot.arm.set_ee_pose_pitch_roll(position, pitch=1.57, roll=0, plan=False, numerical=False)
    #         previous = position
    #     else:
    #         print(f'Smol disp {disp}.')
 

    # robot.arm.set_ee_pose_pitch_roll([0.25, 0., 0.3], pitch=1.57,roll=0, plan=False,numerical=False)
    # # TODO: set position directly in generate_traj?
    # previous = [0.,0.,0.]
    # # TODO: Move out first to prevent hitting robot.
    # for position in trajectory3:
    #     # Move if displacement is significant
    #     disp = np.linalg.norm(np.array(position) - np.array(previous))
    #     if disp > 2.5e-03:
    #         robot.arm.set_ee_pose_pitch_roll(position, pitch=1.57, roll=0, plan=False, numerical=False)
    #         previous = position
    #     else:
    #         print(f'Smol disp {disp}.')
    #     

    # # plan True
    # robot.arm.set_ee_pose_pitch_roll([0.25, 0., 0.3], pitch=1.57,roll=0, plan=False,numerical=False)
    # # TODO: set position directly in generate_traj?
    # previous = [0.,0.,0.]
    # # TODO: Move out first to prevent hitting robot.
    # for position in trajectory:
    #     # Move if displacement is significant
    #     disp = np.linalg.norm(np.array(position) - np.array(previous))
    #     if disp > 2.5e-03:
    #         robot.arm.set_ee_pose_pitch_roll(position, pitch=1.57, roll=0, plan=True, numerical=False)
    #         previous = position
    #     else:
    #         print(f'Smol disp {disp}.')


   # traj 4
    # robot.arm.set_ee_pose_pitch_roll([0.25, 0., 0.35], pitch=1.57,roll=0, plan=False,numerical=False)
    # # TODO: set position directly in generate_traj?
    # previous = [0.,0.,0.]
    # # TODO: Move out first to prevent hitting robot.
    # for position in trajectory4:
    #     # Move if displacement is significant
    #     disp = np.linalg.norm(np.array(position) - np.array(previous))
    #     if disp > 2.5e-03:
    #         robot.arm.set_ee_pose_pitch_roll(position, pitch=1.57, roll=0, plan=False, numerical=False)
    #         previous = position
    #     else:
    #         print(f'Smol disp {disp}.')
    # robot.arm.go_home()

    # # traj 5
    # robot.arm.set_ee_pose_pitch_roll([0.25, 0., 0.35], pitch=1.57,roll=0, plan=False,numerical=False)
    # # TODO: set position directly in generate_traj?
    # previous = [0.,0.,0.]
    # # TODO: Move out first to prevent hitting robot.
    # for position in trajectory5:
    #     # Move if displacement is significant
    #     disp = np.linalg.norm(np.array(position) - np.array(previous))
    #     if disp > 2.5e-03:
    #         robot.arm.set_ee_pose_pitch_roll(position, pitch=1.57, roll=0, plan=False, numerical=False)
    #         previous = position
    #     else:
    #         print(f'Smol disp {disp}.')



    # lift up before ending
    robot.arm.go_home()
