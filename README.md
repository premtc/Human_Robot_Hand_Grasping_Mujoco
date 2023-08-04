
# Allegro Hand Simulation Project

## Overview

This project implements a Proportional Integral Derivative (PID) controller for the Allegro Hand, a robotic hand with 16 individually controllable joints. The Allegro Hand attempts to reach out and grasp objects in a three-dimensional virtual environment that mimics real-world physics using Mujoco, a physics engine for detailed, efficient simulation and to facilitate research in robotics.

## Structure

The project is divided into two main components:

1. `FinalProject.py` - This Python script contains the implementation of the PID controller and the simulation.
2. `Scene/wonik_allegro/scene_right.xml` - This is the Mujoco scene file which includes the 3D model of the Allegro Hand and the environment setup for the simulation. You can find the hand.xml files of the Allegro Hand in the same directory.

## Comamnds to run scene

python3 -m mujoco.viewer --mjcf=/path/to/some/mjcf.xml

## Getting Started

To run the simulation, navigate to the project's directory in your terminal and execute the following command: "mjpython FinalProject.py"

## References

https://github.com/deepmind/mujoco_menagerie
https://mujoco.readthedocs.io/en/stable/python.html
https://www.youtube.com/watch?v=p7wqTpVXug4&list=PLc7bpbeTIk758Ad3fkSywdxHWpBh9PM0G&index=2


## Contact Information

For any further questions or inquiries about the project, feel free to reach out to us:

- Oscar Rivera: osotoriv@gmail.com
- Premt Cara: carapremt@gmail.com or p.cara@tum.de
