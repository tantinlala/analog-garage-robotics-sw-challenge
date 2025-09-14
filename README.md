# Summary

# Packages in workspace
- estop_monitor - Contains a node for publishing time-triggered simulated e-stop events. On startup, the node will immediately publish an e-stop clear message. After a configured period has elapsed, the node will publish an e-stop triggered message.
- proximity_sensor - Contains a node for periodically publishing simulated proximity data sourced from a list as provided in ROS parameters.
- state_machine - Contains a library for an event-driven state machine.
- speed_limiter - Contains a node for publishing speed-limit states based on estop and proximity messages.
- safety_system - Contains a [launch](./src/safety_system/launch/visual_sim_launch.py) file and a parameter [yaml](./src/safety_system/config/visual_sim_params.yaml) file used to bring up all the nodes in a manner where messages are published slowly enough to visually follow a simulated safety system. Also contains an automated [integration test](./src/safety_system/test/test_integration.py) that checks that  speed states are published in the expected order given a deterministic order of messages coming from the estop_monitor and proximity_sensor nodes.

# Architectural docs
- See [node_interactions](./docs/node_interactions.md) for what messages are sent between what nodes.
- See [node_parameters](./docs/node_parameters.md) for descriptions of parameters for each node.
- See [speed_limiter_states](./docs/speed_limiter_states.md) for the state machine diagram for the speed limiter node.

# Code generation tools used
Some amount of code was generated using github copilot, most notably for:
- quickly creating doxygen comments
- for helping to fix compiler errors
- for tab completion

# Instructions

## Setting up development / test environment

The simplest way to set up the development environment is to use vscode's dev containers extension.

0. Make sure that docker and vscode is installed on your host machine.
1. Install the Dev Containers (ms-vscode-remote.remote-containers) extension within vscode.
2. Bring up the command pallete in vscode and run "Dev Containers: Rebuild and Reopen in Container"
3. Wait until the container has started.
4. Upon up a terminal from within vscode to run commands referenced in subsequent sections

The Dockerfile for building the docker image can be found [here](.devcontainer/Dockerfile)

## Building

From the root of the repository, run `./build.sh` to build all packages in the workspace. This will also generate the compile_commands.json file needed for the clangd server to be able to index all source files for easier code navigation.

## Testing 

Testing is performed through a combination of unit tests (written via gtest) and an integration test (using python's launch_testing package).
- Unit tests cover any logic that does not directly depend on the ROS2 API
- The integration test covers logic that does depend on the ROS2 API and checks inter-node interactions/behavior

Run `./test.sh` to run all tests for this workspace while viewing a verbose output of the results.

## Launching simulated safety system

To launch a simulated safety system that updates at a pace suitable for visual verification, run `ros2 launch safety_system visual_sim_launch.py`