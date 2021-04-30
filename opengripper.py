'''
Use this script to insert/ remove tool from the robot gripper. 
type 'open' or 'close' when prompted.
'''

import json
from pyrobot import Robot

robot = Robot('locobot')

# Move to start position.
robot.arm.set_ee_pose_pitch_roll([0.4, 0., 0.3], pitch=1.57, roll=0, plan=True)

def open_gripper():
    robot.gripper.open()

def close_gripper():
    robot.gripper.close()

def move(position):
    robot.arm.set_ee_pose_pitch_roll(position, pitch=1.57, roll=0, plan=True)

def shift_left():
    robot.arm.move_ee_xyz(np.array([-0.5,0,0]))
# while True:
#     command = input("Command (open, close, move, exit): ")
#     if command.lower() == 'open':
#         open_gripper()
#     elif command.lower() == 'close':
#         close_gripper()
#     elif command.lower() == 'move':
#         # move to absolute position
#         position = input('Enter a list [x,y,z]: ')
#         # change string to python list
#         position = json.loads(position)
#         move(position)
#     elif command.lower() == 'exit':
#         robot.arm.set_ee_pose_pitch_roll([0.4, 0., 0.3], pitch=1.57, roll=0, plan=True)
#         break
#     else:
#         print('please enter a valid command.')


# TODO: add GUI for convenience? 
import tkinter as tk

window = tk.Tk()

open_button = tk.Button(
    text="Open",
    width=25,
    height=5,
    bg="black",
    fg="white",
    command=open_gripper
)

# open_button.bind("<Button-1>", open_gripper)

close_button = tk.Button(
    text="Close",
    width=25,
    height=5,
    bg="black",
    fg="white",
    command=close_gripper
)

left_button = tk.Button(
    text="Left",
    width=25,
    height=5,
    bg="black",
    fg="white",
    command=shift_left
)

current_xyz = 

label = tk.Label(text=f"{current_xyz}")

open_button.pack()
close_button.pack()
left_button.pack()

window.mainloop()