# CMU Robot Autonomy Course Project (Team 4)

## Introduction
Using Locobot to draw with a brush  
To try the current drawing:  
1) Run the launch file `roslaunch locobot_control main.launch use_arm:=true torque_control:=false use_rviz:=false`  
2) In another terminal, load the pyrobot environment with `load_pyrobot_env`. Then run `python3 main.py`.  

Example input image: ![Image](/recordings/housenew.png?raw=true "Coordinates of vertices")  

Drawing produced: ![Image](/photos/house1.jpg?raw=true "Robot drawing")  

## Perform your own drawing
`record_traj.py`: Records a human controlled trajectory for a skill. The recording will be saved in /recordings/{your_filename} **Note**: change the ROS argument to `torque_control:=false`  
`replay_traj.py {your_filename}`: Replays the recorded trajectory by end effector position.  
`DMP.py {your_filename}`: Fits a DMP and generate a trajectory. Change the start and end points to test your learned skill.  
`main.py`: Replace the parameters to use your own images and weight files.   

## Others
`opengripper.py`: Convenient script to move arm, open and close gripper.  
`CV/get_vertices.py`: Use this to check the points that will be extracted from the input image.  

Example straight trajectory generated by DMP: ![Plot](/recordings/slant_sav.png?raw=true "Straight line trajectory")  

## Resources
- [Discussion Document](https://docs.google.com/document/d/1hA72cqlCjKWrKFbTAD62o3VZzhTlJhqMhDQjm3AuSK4/edit#)  
- [Locobot docs](https://pyrobot-docs.readthedocs.io/en/latest/core/arm.html)  