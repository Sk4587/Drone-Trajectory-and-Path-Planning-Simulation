# Drone-Trajectory-and-Path-Planning-Simulation

![image](https://github.com/Sk4587/Drone-Trajectory-and-Path-Planning-Simulation/assets/46374770/174e3595-d43a-48c9-bb40-6f2b55240289)

To run the code, in Matlab command window run
```sh
Sim_Quadcopter3
```
### Task

Implement a simulation of a quadcopter, following the non-linear model in [Quadcopter Dynamics Simulation and Control](https://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/)

Assume the entire state of the quadcopter can be measured by sensors with 100% accuracy. Also, assume that the input ϒi to each propeller is limited to +/-1.5. Using the quadcopter simulator from question 1, implement a full-state feedback controller that makes the drone perform the following trajectory:
a. Starts at (0,0,0) 
b. Moves up to (0,0,5) 
c. Stays at (0,0,5) for 5 seconds. 
d. Moves to (0,2.5,5)
e. Moves along a circular trajectory with radius 2.5 on a vertical plane with constant x=0, passing through the points (0,0,7.5), (0,-2.5,5), (0,0,2.5) and back to (0,2.5,5).
f. Moves to (2.5,2.5,2.5)
g. Lands at (2.5,2.5,0) safely by moving down at a constant speed of 0.1 m/s.

More details about the task are given [here](https://github.com/Sk4587/Drone-Trajectory-and-Path-Planning-Simulation/blob/main/QuadcopterCoursework.pdf)

### Approach
To implement the full-state feedback system, I first checked the reachability of the discretised A and B matrices I got from Question 1 using the MATLAB ‘ctrb’ function. The system is fully reachable.
<br>
I used the pole placement method to design the controller to get the feedback K matrix. The Eigenvalues are chosen such that the eigenvalues corresponding to the Quadcopter’s position and angular orientation are closer to 1 (dominant poles), and all the other values are around 0.8. 
<br>
The fsf controller is based on the linear approximation of the system. The new input corresponding to the reference is given as
<br>
$$u= -fsf_k*(current state-reference state) +u0$$
<br>
This input is then given to the non-linear model, and the input is accounted for the deviation of equilibrium u0. 
<br>
The drone will follow the required trajectory by updating the reference state at each point. 
<br>
I use a slightly higher threshold at the circular trajectory to get a smoother and faster curve.
<br>
<br>
To land the drone safely from (2.5,2.5,2.5) to (2.5,2.5,0) at a constant speed of 0.1, I updated the reference state as (2.5,2.5, Current Z position -0.11, 0,0,-0.11, zeros(6)) so that the reference is updated smoothly, thereby controlling the velocity.

![image](https://github.com/Sk4587/Drone-Trajectory-and-Path-Planning-Simulation/assets/46374770/96bd8289-22b0-4eae-ab6d-09c8e533c454)
<b>Note:</b> This project is part of Coursework for COMP0128-Robotic Control Theory and Systems at University College London. Copyrights under [Dr. Francisco Vasconcelos](https://www.ucl.ac.uk/surgical-robot-vision/francisco-vasconcelos)



