'''
Use this script to insert/ remove tool from the robot gripper. 
Press open/close and wait for execution to finish. 
TODO: display the xyz position of the end effector with live update
'''

import json
import tkinter as tk
import numpy as np

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

def moveto():
    pos = f'[{pos_var.get()}]'
    robot.arm.set_ee_pose_pitch_roll(json.loads(pos), pitch=1.57, roll=0, plan=True)

# Entry to move arm
window = tk.Tk()

pos_var = tk.StringVar(value='0.3,0,0.3')
pos_entry = tk.Entry(text='Desired pos',textvariable=pos_var)

def moveto():
    pos = f'[{pos_var.get()}]'
    robot.arm.set_ee_pose_pitch_roll(json.loads(pos), pitch=1.57, roll=0, plan=True)
    # xyz_var.set(str(robot.arm.pose_ee[0]))
    print(robot.arm.pose_ee[0])

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

move_button = tk.Button(
    text="MoveTo",
    width=25,
    height=5,
    bg="black",
    fg="white",
    command=moveto
)

xyz_var = tk.StringVar()
xyz_var = str(robot.arm.pose_ee[0])
# current_xyz = robot.arm.pose_ee[0]
xyz_label = tk.Label(textvariable=xyz_var)

xyz_label.pack()
open_button.pack()
close_button.pack()
pos_entry.pack()
move_button.pack()

window.mainloop()


## CLI command version
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
