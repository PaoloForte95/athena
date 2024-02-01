# Athena

<p align="center">
  <img height="300" src="doc/logo.png" />
</p>

Athena is a ROS2 planning framework based on behavior trees. Its main features are:

* Compute execution plan for problem defined with PDDL 3.1 and HDDL 1.0.
* Compute total ordered, partial ordered, and concurrent plans.
* Dispatch and execute the actions using behavior tree.

# Overview

# Getting Started
First, install <a href="https://protobuf.dev/overview/">protobuf</a>.

## Installation
To install, clone this repository and build the source code with colcon:
```
$ git clone https://gitsvn-nt.oru.se/pofe/athena.git
$ cd athena
$ colcon build --packages-up-to athena
```
To build the task planner executable that calls the Java code to parse the planning problem and compute the execution plan:
```
$ cd src/athena/athena_protobuf/
$ ./gradlew build
```
This will create an executable .jar file into athena_planner/Planners called task_planner.

## Running an example

1) Start a terminal in your GUI
2) Source the setup file
```
source install/setup.bash
```
3) In the same terminal, run:
```
ros2 launch athena_launch planning.py
```
4) Open a new terminal, souce the setup file, and run: 
```
ros2 topic pub --once /planning_problem athena_msgs/msg/PlanningProblem "{planning_domain: src/athena/athena_example/HDDL/construction/domains/domain.hddl, planning_problem: src/athena/athena_example/HDDL/construction/problems/pfile01.hddl}"

```

# Add your own planner
The framework facilitates the easy integration of new task planners into the list of selectable planners.