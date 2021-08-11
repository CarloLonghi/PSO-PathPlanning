PATH PLANNER USING PARTICLE SWARM OPTIMIZATION

Simple Particle Swarm Optimization applied to the task of path planning.
The goal is to find the shortest path between two points in a map with some obstacles.
A path represents a particle in the PSO algorithm. A customizable number of consecutive checkpoints describes every path. Every path is made by the segments between those checkpoints, the starting point and the goal.

The Particle Swarm Optimization has been improved adding some randomization and adaptiveness to the inertia weight.
Random values are assigned to the velocities of the particles at a fixed interval and if the global best values doesn't improve after many iterations.
The inertia weight of the particle velocities is initially small and is increased as the iteration number gets bigger.
This strategy helps to favor exploration during the first iterations and exploitation on the last iterations.

The obstacles size and position are generated randomly at every run of the program.


INSTRUCTIONS

To test the program run the PSO.py file.
Environment and PSO parameters can be set directly in the code.

![First Example Animation](./images/anim3.gif)
![First Example Final Path](https://github.com/CarloLonghi/PSO-PathPlanning/tree/main/images/final_path3.png)
![Second Example Animation](https://github.com/CarloLonghi/PSO-PathPlanning/tree/main/images/anim4.gif)
![Second Example Final Path](https://github.com/CarloLonghi/PSO-PathPlanning/tree/main/images/final_path4.png)
![Third Example Animation](https://github.com/CarloLonghi/PSO-PathPlanning/tree/main/images/anim5.gif)
![Third Example Final Path](https://github.com/CarloLonghi/PSO-PathPlanning/tree/main/images/final_path5.png)
![Fourth Example Animation](https://github.com/CarloLonghi/PSO-PathPlanning/tree/main/images/anim6.gif)
![Fourth Example Final Path](https://github.com/CarloLonghi/PSO-PathPlanning/tree/main/images/final_path6.png)
![Fifth Example Animation](https://github.com/CarloLonghi/PSO-PathPlanning/tree/main/images/anim7.gif)
![Fifth Example Final Path](https://github.com/CarloLonghi/PSO-PathPlanning/tree/main/images/final_path7.png)
