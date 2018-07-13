# robust_multirobot_map_merging

This repository contains an implementation of the robust map merging method presented in
```
J. G. Mangelson, D. Dominic, R. M. Eustice, and R. Vasudevan, 
“Pairwise Consistent Measurement Set Maximization for Robust Multi-robot Map Merging.”, ICRA 2018.
```

# Usage with ROS
- Clone this repository in the source folder of your workspace.
- Build using `catkin`.
- Launch with `rosrun robust_multirobot_map_merging robust_multirobot_map_merging_node <trajectory robot1 .g2o file> <trajectory robot2 .g2o file> <inter robot loop closures .g2o file> <output file (optional)>`.