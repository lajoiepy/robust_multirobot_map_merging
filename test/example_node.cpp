// author: Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "graph_utils.h"
#include "pairwise_consistency.h"

#include <string>
#include <iostream>
#include <eigen3/Eigen/Geometry>

#define THRESHOLD 1.635

int main(int argc, char* argv[])
{
  // Parse arguments
  std::string input_file_name, output_file_name;
  if (argc < 2) {
    std::cout << "Not enough arguments, please specify an input file. (format supported : .g2o)" << std::endl;
    return -1;
  } else {
    input_file_name = argv[1];
    if (argc > 2) {
      output_file_name = argv[2];
    }
    else {
      output_file_name = "consistency_matrix.clq.mtx";
    }
  }
  // Preallocate output variables
  size_t num_poses;
  std::map<std::pair<size_t,size_t>, graph_utils::Transform> transforms;
  std::list<std::pair<size_t,size_t>> loop_closure_list;

  // Parse the graph file
  std::cout << "Parsing of file : " << input_file_name << std::endl;
  graph_utils::parseG2ofile(input_file_name, num_poses, transforms, loop_closure_list);
  
  // Compute the non-optimized trajectory
  std::cout << "Trajectory computation." << std::endl;
  std::map<size_t, graph_utils::TrajectoryPose> trajectory = graph_utils::buildTrajectory(transforms);

  // Compute the pairwise consistency
  std::cout << "Pairwise consistency computation." << std::endl;
  robust_multirobot_slam::PairwiseConsistency pairwise_consistency(transforms, loop_closure_list, trajectory);
  Eigen::MatrixXi consistency_matrix = pairwise_consistency.computeConsistentMeasurementsMatrix(THRESHOLD);

  // Print the result to compute the maximum clique in a file
  std::cout << "Print result in " << output_file_name << std::endl;
  graph_utils::printConsistencyGraph(consistency_matrix, output_file_name);

  return 0;
}