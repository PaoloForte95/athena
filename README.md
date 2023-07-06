# Athena

Athena is a ROS2 planning framework based on behavior trees. Its main features are:

* Compute execution plan for problem defined with PDDL 3.1 and HDDL 1.0.
* Compute total ordered, partial ordered, and concurrent plans.
* Dispatch and execute the actions using behavior tree.

<p align="center">
  <img height="300" src="doc/logo.png" />
</p>

# Overview

# Getting Started

## Installation
To install, clone this repository and build the source code with colcon:
```
$ git clone https://gitsvn-nt.oru.se/pofe/athena.git
$ cd athena
$ colcon build --packages-up-to athena
```
To build the task planner executable that calls the Java code to parse the planning problem and compute the execution plan:
```
$ cd src/athena/plan2_protobuf/
$ ./gradlew build
```
This will create an executable .jar file into plan2_planner/Planners to call the task planner.

## Running an example

1) Start a terminal in your GUI
2) Source the setup file
```
source install/setup.bash
```
3) In the same terminal, run:
```
ros2 launch plan2_launch planning.py
```
4) Open a new terminal
```
source install/setup.bash
ros2 topic pub --once /planning_problem plan2_msgs/msg/PlanningProblem "{planning_domain: /home/paolodell/dev_ws/ros2_humble_ws/src/athena/plan2_example/PDDL/Domain.pddl, planning_problem: /home/paolodell/dev_ws/ros2_humble_ws/src/athena/plan2_example/PDDL/Problem.pddl}"
```