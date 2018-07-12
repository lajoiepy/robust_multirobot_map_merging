#include "graph_utils.h"
#include "robust_multirobot_slam.h"

#include <string>
#include <iostream>
#include <eigen3/Eigen/Geometry>

int main(int argc, char* argv[])
{
  // Preallocate output variables
  size_t num_poses;
  std::map<std::pair<size_t,size_t>, graph_utils::Transform> transforms;
  std::list<std::pair<size_t,size_t>> loop_closure_list;

  graph_utils::parseG2ofile("/home/lajoiepy/Documents/master/mit/graph_data/CSAIL.g2o", num_poses, transforms, loop_closure_list);
  
  std::map<size_t, graph_utils::TrajectoryPose> trajectory = graph_utils::buildTrajectory(transforms);

  Eigen::MatrixXi consistency_matrix = robust_multirobot_slam::computeConsistentMeasurementsMatrix(transforms, loop_closure_list, trajectory);

  graph_utils::printConsistencyGraph(consistency_matrix);

  return 0;
}