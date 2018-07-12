// author: Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "graph_utils.h"
#include "pairwise_consistency.h"

#include <string>
#include <iostream>
#include <eigen3/Eigen/Geometry>

int main(int argc, char* argv[])
{
  // Preallocate output variables
  size_t num_poses;
  std::map<std::pair<size_t,size_t>, graph_utils::Transform> transforms;
  std::list<std::pair<size_t,size_t>> loop_closure_list;

  // Parse the graph file
  graph_utils::parseG2ofile("/home/lajoiepy/Documents/master/mit/graph_data/CSAIL.g2o", num_poses, transforms, loop_closure_list);
  
  // Compute the non-optimized trajectory
  std::map<size_t, graph_utils::TrajectoryPose> trajectory = graph_utils::buildTrajectory(transforms);

  // Compute the pairwise consistency
  robust_multirobot_slam::PairwiseConsistency pairwise_consistency(transforms, loop_closure_list, trajectory);
  Eigen::MatrixXi consistency_matrix = pairwise_consistency.computeConsistentMeasurementsMatrix();

  graph_utils::printConsistencyGraph(consistency_matrix);

  return 0;
}