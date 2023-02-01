# real-world-bench

This repository contains set of tools intended for testing and benchmarking navigational approaches in dynamic real robot scenarios. Our solution employs virtual obstacles and ray intersections to manpiulate the existing scan data from the robot to include the virtual obstacles, making it seem as though they also exist in the real world.

# Demo


# Usage
The laser_manipulation node depends on a source of virtual obstacles to be published under **/simulated_agents** in the [**pedsim_msgs::AgentStates**](https://github.com/Arena-Rosnav/pedsim_ros/tree/master/pedsim_msgs/msg) format. We provide two different options as sources of obstacles:
- [arena-tools](https://github.com/Arena-Rosnav/arena-tools) scenario
- [Pedsim](https://github.com/Arena-Rosnav/pedsim_ros) simulation

## Arena-tools
Generic program call
```bash
roslaunch real-world-bench start_arena_bench.launch scenario_path:=[path]
```
## Arena-tools
