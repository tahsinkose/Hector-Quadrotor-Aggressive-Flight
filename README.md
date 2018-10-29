# Hector-Quadrotor-Aggressive-Flight
Implementation of the algorithms offered in the paper <a href="http://ai.stanford.edu/~gabeh/papers/GNC08_QuadTraj.pdf">Quadrotor Helicopter Trajectory Tracking Control</a> originally  for the STARMAC platform. In this case, it is implemented for Hector Quadrotor using ROS as the framework. Outputs of several patterns and trajectories are taken from RViz.

## Usage

It is assumed that Hector Quadrotor stack is properly built in the environment. With that pre-requisite met: 

1- Run in the first terminal:

`roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch`

2- Run in the second terminal:

`rosrun hector_aggressive_flight trajectory_controller`

Program is tested in Ubuntu 16.04 with ROS Kinetic. Nonetheless, other frameworks would not possess a problem.

## To-Do

Currently, cross-track acceleration input is not implemented. Therefore, the error between the actual position and the goal position is generally 1-3 meters.
With that, errors should highly reduce. Nevertheless; since this system is an open-loop, the errors inevitably would accumulate in bigger trajectories. Hence, a closed loop controller
that regulates velocities through fetching odometry information might be needed. However, the odometry callback might have a problematic timing regarding to correct navigation since both the velocities and accelerations are computed as timed entities. In brief, it must be really fast.
