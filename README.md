# robot_control

This project is task based and involes the open loop simulation and contoller implementation of the following two-link manipulator:

[Two-Link Manipulator](two_link_manipulator.PNG)

For the open loop simulation, the task is simply to simulate the two-link manipulator's natural response to three different initializations, where the initial position of the manipulator, the manipulator's motor torques and the friction acting on the manipulator are varied with each. 

For the controller implimentation, the task is then to design two seperate controllers and simulate the performance of both with regards to moving the manipulator from an initial position to a given set point. The first controller that is designed is a PD + gravity controller, which is an example of a closed loop joint-space controller. The second designed controller is then a stiffness cotroller, which is an example of a closed loop Cartesian-space controller.

I completed this project for a course I took during my master's degree on control in robotics. The specific tasks are outlined in the file "assgt5_2018.pdf[Two-Link Manipulator](assgt5_2018.pdf)". The project write up can then be found in the file "J.Morrison_A5.pdf".
