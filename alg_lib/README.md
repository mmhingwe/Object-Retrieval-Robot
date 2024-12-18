This project aims to showcase the skills I learned in school (Computer Vision, AI, Controls). The end goal of this project is to have a robot go to 
a specified location and pick up and object and return it autonomously. 

So far, I have implemented a basic RRT algorithm to find a path. My next step is to create a class to define the dynamics and kinematics of a robot manipulator 
using twists. Then create a PID controler to have the arm move to a goal point. I will implement more complex controls such as MPC in the future. Additionally, I plan to implement RRT* using KDTrees (to speed up finding the nearest neighbor). 
