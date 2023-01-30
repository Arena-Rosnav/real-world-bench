# real-world-bench

This repository contains set of tools intended for testing and benchmarking navigational approaches in dynamic real robot scenarios. Our solution employs virtual obstacles and ray intersections to manpiulate the existing scan data from the robot to include the virtual obstacles, making it seem as though they also exist in the real world.

# Demo


# Usage
The laser_manipulation node depends on a source of virtual obstacles to be published under **/simulated_agents** in the **TODO:MSG_FORMAT** type. 
We provide two different options:
- [arena-tools]() scenario
- [Pedsim]() simulation
