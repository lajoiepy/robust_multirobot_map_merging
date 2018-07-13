// author: Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "graph_utils.h"
#include "pairwise_consistency.h"
#include "findClique.h"

#include <string>
#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <chrono>

#define THRESHOLD 1.635

const std::string CONSISTENCY_MATRIX_FILE_NAME = "consistency_matrix.clq.mtx";

int main(int argc, char* argv[])
{
  std::cout << "---------------------------------------------------------" << std::endl;
  // Parse arguments
  std::string robot1_file_name, robot2_file_name, interrobot_file_name, output_file_name;
  if (argc < 4) {
    std::cout << "Not enough arguments, please specify at least 3 input files. (format supported : .g2o)" << std::endl;
    return -1;
  } else {
    robot1_file_name = argv[1];
    robot2_file_name = argv[2];
    interrobot_file_name = argv[3];

    if (argc > 4) {
      output_file_name = argv[4];
    }
    else {
      output_file_name = "results.txt";
    }
  }

  // Preallocate output variables
  size_t num_poses_robot1, num_poses_robot2, num_poses_interrobot;
  graph_utils::TransformMap transforms_robot1, transforms_robot2, transforms_interrobot;
  std::list<std::pair<size_t,size_t>> loop_closure_list;

  // Parse the graph file
  std::cout << "Parsing of the following files : " << robot1_file_name << ", " << robot2_file_name << ", " << interrobot_file_name;
  auto start = std::chrono::high_resolution_clock::now();
  graph_utils::parseG2ofile(robot1_file_name, num_poses_robot1, transforms_robot1, loop_closure_list, false);
  graph_utils::parseG2ofile(robot2_file_name, num_poses_robot2, transforms_robot2, loop_closure_list, false);
  graph_utils::parseG2ofile(interrobot_file_name, num_poses_interrobot, transforms_interrobot, loop_closure_list, true);
  auto finish = std::chrono::high_resolution_clock::now();
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;
  
  // Compute the non-optimized trajectory
  std::cout << "Trajectory computation." ;
  start = std::chrono::high_resolution_clock::now();
  auto trajectory_robot1 = graph_utils::buildTrajectory(transforms_robot1);
  auto trajectory_robot2 = graph_utils::buildTrajectory(transforms_robot2);
  finish = std::chrono::high_resolution_clock::now();
  milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;

  // Compute the pairwise consistency
  std::cout << "Pairwise consistency computation.";
  start = std::chrono::high_resolution_clock::now();
  robust_multirobot_slam::PairwiseConsistency pairwise_consistency(transforms_robot1, transforms_robot2, transforms_interrobot, loop_closure_list, trajectory_robot1, trajectory_robot2);
  Eigen::MatrixXi consistency_matrix = pairwise_consistency.computeConsistentMeasurementsMatrix(THRESHOLD);
  finish = std::chrono::high_resolution_clock::now();
  milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;

  // Print the result to compute the maximum clique in a file
  std::cout << "Print result in " << CONSISTENCY_MATRIX_FILE_NAME;
  start = std::chrono::high_resolution_clock::now();
  graph_utils::printConsistencyGraph(consistency_matrix, CONSISTENCY_MATRIX_FILE_NAME);
  finish = std::chrono::high_resolution_clock::now();
  milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;

  // Pass results to fast max-clique finder library
  std::cout << "Compute max-clique problem";
  start = std::chrono::high_resolution_clock::now();
  CGraphIO gio;
  gio.readGraph(CONSISTENCY_MATRIX_FILE_NAME);
  int iMaxClique = 0;
  std::vector<int> max_clique_data;
  iMaxClique = maxClique(gio, iMaxClique, max_clique_data);
  std::cout << " - Max clique Size : " << iMaxClique;
  finish = std::chrono::high_resolution_clock::now();
  milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;
  print_max_clique(max_clique_data);
  max_clique_data.clear();

  // Reassign result and print consistent loop closures in output file
  

  return 0;
}