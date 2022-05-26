# Laboratory of Applied Robotics Student Interface
Package used by student to complete the assignment of the course. 

## Project objectives

    development and implementation of sensing algorithms to recognise objects and obstacles in the competition field
    development and implementation of motion planning algorithms to move the robot from an initial to a final position in minimum time avoiding obstacles
    development and implementation of intelligent planning algorithms to decide the optimal game strategy
    testing of the solution in simulation and on the field

## Installation

Just execute the "install_lab.sh" script. It will install ROS, Gazebo, the planner and all the necessary dependecies.

## Run the project
Open terminator and split it in 3 terminals. Move in workspace/project/build folder and use the command "make" to compile the project.
In a terminal use the command "AR_simulator_gui" (or "AR_simulator" if you don't want the gui) to start the simulator. In the second terminal use "AR_pipeline" to start the pipeline. If it is necessary to specify the number of robots present, it must be done both at the start of the simulator and the pipeline with the option "n:=NumberOfRobots" (Ex. AR_simulator_gui n:=2, AR_pipeline n:=2). In the last terminal type AR_run to execute the program.
