# Robust Multi-Robot Map Merging

This repository contains an implementation of the robust map merging method presented in
```
J. G. Mangelson, D. Dominic, R. M. Eustice, and R. Vasudevan, 
“Pairwise Consistent Measurement Set Maximization for Robust Multi-robot Map Merging.”, ICRA 2018.
```

The **purpose of this package** is to find a subset of inliers in a large set of inter-robot loop closures. Due to large measurement errors and perceptual aliasing in multi-robot applications, we need a **robust method** to select which measurements to trust.

## Requirements
- **geometry_msgs**   (http://wiki.ros.org/geometry_msgs)
- **MRPT**            (https://www.mrpt.org/)
- **mrpt_bridge**     (http://wiki.ros.org/mrpt_bridge)

## Example
An example of usage of the libraries is available in the `examples/` folder.

## Build
- Clone this repository in the source folder of your ROS workspace.
- Build using `catkin build` or `catkin_make`.

_You can also build it normally using cmake._

## Try It!
- Launch with `rosrun robust_multirobot_map_merging robust_multirobot_map_merging_node <trajectory robot1 .g2o file> <trajectory robot2 .g2o file> <inter robot loop closures .g2o file>`

_Some .g2o files are available for testing in the `pose_graph_datasets/` folder._
