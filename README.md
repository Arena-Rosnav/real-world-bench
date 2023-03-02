# real-world-bench

This repository contains set of tools intended for testing and benchmarking navigational approaches in dynamic real robot scenarios. Our solution employs virtual obstacles and ray intersections to manpiulate the existing scan data from the robot to include the virtual obstacles, making it seem as though they also exist in the real world.

# Demo



https://user-images.githubusercontent.com/41898845/222550808-0f03a282-956a-4cea-8728-65ae6518c9c2.mp4



# Usage
The laser_manipulation node depends on a source of virtual obstacles to be published under **/simulated_agents** in the [**pedsim_msgs::AgentStates**](https://github.com/Arena-Rosnav/pedsim_ros/tree/master/pedsim_msgs/msg) format. We provide two different options as sources of obstacles:
- [arena-tools](https://github.com/Arena-Rosnav/arena-tools) scenario
- [Pedsim](https://github.com/Arena-Rosnav/pedsim_ros) simulation

## Arena-tools
Generic program call
```bash
roslaunch laser_manipulation start_arena.launch scenario_path:=[path] #Path to a specific scenario.json file needs to be given
```
## Pedsim
The manipulation node assumes, that a working pedsims simulation is already launched and publishing obstacles information, so you need to make sure that pedsim is running before using this call.
```bash
roslaunch laser_manipulation start_pedsim.launch
```
## General program arguments
| Name             | Default                 | Type             | Description                                                                                                                                                                                                                                                        |
| ---------------- | ----------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| laser_scan_topic            | scan                 | string              | Name of the topic, on which the robot is publishing scan data                                                                                                                                                                                                                             |
| output_topic    | scan_manipulated                     |        string          | Topic under, which manipulated scan will be published                                                                                                                                                                                                                               |
| radius      | 0.1                | double | Defines the radius of each virtual obstacle model                                                                                                                                                                                                                                          |
| viz |              true           | bool           | Whether visualization of obstacles should be published |

