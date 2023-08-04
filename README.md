# Human_Robot_Hand_Grasping
Final project

## Task:

### Exercise:
#### Todos init:
- Run your PC with Ubuntu 
- Install ROS (if needed)
- Create a object with physical parameters model in Mujoco
- Install the target hand (Allegrom shadow …or other suitable hand) with Mujoco
  - Allegro Hand
  -	Seed Robotics RH8D (https://drive.google.com/drive/folders/1LjQT571LgDIuNELxmXbR3TLphAKbCZpB?usp=sharing)
  - Shadow Hand
  -	Prensilia hand ih2 azzurra
- For the Seed Robotics Hand you can find the URDF files in Rocket chat. You can use this ros plugin to convert it to a mujoco model. Useful links:
  - https://github.com/wangcongrobot/dual_ur5_husky_mujoco
  - https://wiki.ros.org/urdf
  - https://github.com/ros/urdfdom


#### Todos later:
- Simulation
- Define the hand to be used in Mujoco (Jacobian? Grasp matrix)
- Define the contacts (SF, HF, FF)
- Create the scene, and object physics t.d.f
- Use the following paper as a reference (see Fig. 12 5) You can choose 3 grasps from these 5 and choose objects that trigger those grasps https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6822601&casa_token=mC3UZ-czOc4AAAAA:9siWTLjTcNaChtYrS_ZxoL0dttodRYOhj1ore3_vt7_ieQGsU_Bdt1q5BduHh_VikfZUzdjCZw&tag=1
- Achieving stable grasp: gasp and lift
- For your controllers, you can choose the ones that suit better, impedance, PID position control etc. Please reason why you chose that certain type of controller and the benefits of it. Please remember that the hand motion should not be hard coded, but rather result from your control algorithm.
#### Hardware (bonus points, optional)
- Implement all the above simulation controllers to the real hand
-	You can try to use the Seed Robotics Hand of Prensilia hand for this. 

## Comamnds to run scene

python3 -m mujoco.viewer --mjcf=/path/to/some/mjcf.xml

## References

https://github.com/deepmind/mujoco_menagerie
https://mujoco.readthedocs.io/en/stable/python.html
https://www.youtube.com/watch?v=p7wqTpVXug4&list=PLc7bpbeTIk758Ad3fkSywdxHWpBh9PM0G&index=2

# Allegro Hand Simulation Project

## Overview

This project implements a Proportional Integral Derivative (PID) controller for the Allegro Hand, a robotic hand with 16 individually controllable joints. The Allegro Hand attempts to reach out and grasp objects in a three-dimensional virtual environment that mimics real-world physics using Mujoco, a physics engine for detailed, efficient simulation and to facilitate research in robotics.

## Structure

The project is divided into two main components:

1. `FinalProject.py` - This Python script contains the implementation of the PID controller and the simulation.
2. `Scene/wonik_allegro/scene_right.xml` - This is the Mujoco scene file which includes the 3D model of the Allegro Hand and the environment setup for the simulation. You can find the hand.xml files of the Allegro Hand in the same directory.

## Getting Started

To run the simulation, navigate to the project's directory in your terminal and execute the following command: "mjpython FinalProject.py"

## Contact Information

For any further questions or inquiries about the project, feel free to reach out to us:

- Oscar Rivera: osotoriv@gmail.com
- Premt Cara: carapremt@gmail.com or p.cara@tum.de
