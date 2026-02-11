<h1 align="center">ATHENA</h1>

<p align="center">
  <img height="300" src="doc/logo.png" />
</p>

Athena is a planning framework based on behavior trees. Its main features are:

* Compute execution plan for problem defined with PDDL 3.1 and HDDL 1.0.
* Compute total ordered, partial ordered, and concurrent plans.
* Plan, dispatch, and execute the actions using behavior tree.

<h1 align="center">Overview</h1>

Athena is composed of two main components: **planning** and **execution**.

The planning component is responsible for parsing the domain and problem descriptions and computing an execution plan. The execution component dispatches and executes the actions and methods contained in the plan. Since actions are implemented as behavior tree nodes, their execution logic can be customized and extended.


<h1 align="center">Dependencies</h1>

# Protobuf
Install Protobuf from https://github.com/protocolbuffers/protobuf


# PDDL4j
Clone and build PDDL4J. It is not necessary to build it inside the ROS 2 workspace.
```
git clone https://github.com/PaoloForte95/pddl4j
cd pddl4j
git checkout devel
./gradlew build -PnoCheckStyle -PnoTest
```
If you encounter issues related to the Java version, update the distributionUrl in the gradle-wrapper.properties file according to your java version. You can check the Compatibility Matrix [here] (https://docs.gradle.org/current/userguide/compatibility.html):
```
distributionUrl=https\://services.gradle.org/distributions/<compatible_gradle_version>-bin.zip
```
<h1 align="center">Installation</h1>

Clone the Athena repository:
```
$ git clone https://github.com/PaoloForte95/athena.git
```
Copy the generated PDDL4J JAR file into the Athena protobuf library directory:
```
$ cp <path/to/pddl4j>/build/libs/pddl4j-4.0.0.jar src/athena/planning/athena_protobuf/lib/
```
Build the task planner executable, which invokes the Java code for parsing planning problems and computing execution plans:
```
$ cd src/athena/planning/athena_core/
$ ./gradlew build
$ cp <path/to/pddl4j>/build/libs/athena_core-0.1.0 src/athena/planning/athena_protobuf/lib/
$ cd ../athena_protobuf/
$ ./gradlew build
```
If you encounter issues related to the Java version, update the distributionUrl in the gradle-wrapper.properties file according to your java version. 

This process generates an executable JAR file named task_planner in athena_planner/Planners. This executable is used to compute execution plans.

The parsing logic and the concurrent-plan generation code are located in athena_core. Any modification to this component requires rebuilding the project.

Finally, build the ROS 2 workspace using colcon
```
$ colcon build --packages-up-to athena
```

<h1 align="center">Running</h1>

To run an example

1) Source the workspace:
```
source install/setup.bash
```
2) Launch the planner:
```
ros2 launch athena_launch planning_launch.py
```
3) In a new terminal, launch the executor:
```
source install/setup.bash
ros2 launch athena_exe_launch execution_launch.py
```
4) In a new terminal, source the workspace again and publish a planning problem:
```
source install/setup.bash
ros2 topic pub --once /planning_problem athena_msgs/msg/PlanningProblem "{planning_domain: <path/to/pddl/planning/domain>, planning_problem: <path/to/pddl/planning/problem>}"
```

<h1 align="center">Connection with VLM</h1>

# Python Virtual Environment

Create a <a href="https://docs.python.org/3/library/venv.html">python virtual environment.</a>. 

Activate the environment:
```
source <path/to/venv/bin/activate>
```
Install the required dependencies:
```
pip install openai "numpy==1.26.4" "opencv-python==4.10.0.84" google-genai pillow
```

# Set up for api keys.
These keys are required for automatic planning-problem generation. <br> 
Add the following lines to your .bashrc:
```
export OPEN_API_KEY="your_api_key_here"
expor GEMINI_API_KEY="your_api_key_here"
```
# Creating a Prompt
Athena can automatically generate planning problem files. To enable this feature, create a prompt file and save it in the ROS 2 workspace.

<h1 align="center">Add your own planner</h1>

Athena supports the integration of additional task planners.

To add a new planner:

1) Download the planner or create a JAR file with a main entry point and place it in the the athena_planner/Planners directory.

2) Create a plugin definition for the new planner so it can be selected by the framework.
