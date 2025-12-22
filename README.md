# PROST ROS 1 Wrapper

This package provides a ROS 1 wrapper for the [PROST](https://github.com/prost-planner/prost) probabilistic planner. It allows ROS nodes to utilize PROST for online planning and execution in RDDL domains.

## Overview

The core node `prost_bridge.py` acts as a bridge between the ROS ecosystem and the PROST planner. It mocks the RDDLSim server protocol, enabling PROST to communicate with your robot or simulation via standard ROS services.

### Workflow
1.  **Start Planning**: A ROS client sends a domain and instance description (RDDL) to the bridge.
2.  **Initialization**: The bridge launches PROST in a subprocess.
3.  **Execution Loop**:
    *   Robot sends current **Observations** (state fluents) to the bridge.
    *   Bridge forwards them to PROST.
    *   PROST calculates the next **Action**.
    *   Bridge returns the action to the robot.
    *   Robot executes the action and repeats.

## Prerequisites

### System Dependencies
You must install the necessary build tools and libraries for specificall for PROST:
```bash
sudo apt update
sudo apt install git g++ cmake bison flex libbdd-dev libz3-dev
```
*Note: `libz3-dev` (Z3 Theorem Prover) is critical.*

### ROS Dependencies
*   ROS 1 (Noetic recommended)
*   `rospy`
*   `std_msgs`

## Build Instructions

1.  **Build PROST**
    Navigate to the `prost_ros/prost` directory and build the planner:
    ```bash
    cd prost
    ./build.py
    ```
    *Troubleshooting*: If you see CMake errors about cached directories, remove the `builds` folder and try again (`rm -rf builds && ./build.py`).

2.  **Build ROS Package**
    Ensure this package is inside your Catkin workspace's `src` folder.
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## Usage

### 1. Run the Bridge Node
Start the bridge, specifying the path to the `prost.py` script.

```bash
rosrun prost_ros prost_bridge.py _prost_path:=/path/to/prost/prost.py
```
*Note*: _prost_path:=~/catkin_ws/src/prost_ros/prost/prost.py

*   **Parameters: (No need to change if running all on same machine)**
    *   `host` (default: 127.0.0.1): TCP host for PROST connection.
    *   `port` (default: 2323): TCP port.
    *   `prost_path`: Absolute path to the `prost.py` executable.

### 2. Run Demos
We provide several example clients to demonstrate usage.

**Navigation Domain:**
Simulates a robot navigating a grid world.
```bash
cd ~/catkin_ws/src/prost_ros/prost/testbed/benchmarks/navigation-2011
rosrun prost_ros navigation_demo.py
```
or
```bash
cd ~/catkin_ws/src/prost_ros/prost/testbed/benchmarks/navigation-2011
rosrun prost_ros navigation_demo_v2.py
```
in V2 demo, we are simulating unsecessful robot action by not moving the robot in every other simulation step.

## ROS API

### Services

#### `/prost_bridge/start_planning`
Starts a new planning session.
*   **Request**:
    *   `string domain_content`: Full text of the domain RDDL file.
    *   `string instance_content`: Full text of the instance RDDL file.
    *   `int32 timeout`: Planning timeout in seconds.
*   **Response**:
    *   `bool success`

#### `/prost_bridge/submit_observation`
Submits the current state and retrieves the next action.
*   **Request**:
    *   `KeyValue[] observation`: List of state fluents (e.g., `robot-at(x,y)` = `true`).
    *   `float32 reward`: Reward received from the *previous* action.
*   **Response**:
    *   `string action_name`: Name of the action to execute.
    *   `string[] action_params`: Parameters for the action.

## Creating Your Own Client

To use PROST with your own robot:
1.  Create a ROS node that acts as the "Environment".
2.  Call `start_planning` with your RDDL files.
3.  In a loop:
    *   Gather state from your robot.
    *   Convert state to a list of `KeyValue` objects (matching your RDDL `state-fluents`).
    *   Call `submit_observation`.
    *   Execute the returned action.

## Known Issues
*   **"Text file busy" on reset**: If you restart a planning session quickly, you might see an error because the previous PROST process hasn't fully released the file lock. Wait a few seconds or restart the `prost_bridge` node.
