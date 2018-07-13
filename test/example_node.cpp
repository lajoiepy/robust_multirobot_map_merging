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
      output_file_name = "results.txt";
    }
  }
  // Preallocate output variables
  size_t num_poses;
  std::map<std::pair<size_t,size_t>, graph_utils::Transform> transforms;
  std::list<std::pair<size_t,size_t>> loop_closure_list;

  // Parse the graph file
  std::cout << "Parsing of file : " << input_file_name;
  auto start = std::chrono::high_resolution_clock::now();
  graph_utils::parseG2ofile(input_file_name, num_poses, transforms, loop_closure_list);
  auto finish = std::chrono::high_resolution_clock::now();
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;
  
  // Compute the non-optimized trajectory
  std::cout << "Trajectory computation." ;
  start = std::chrono::high_resolution_clock::now();
  std::map<size_t, graph_utils::TrajectoryPose> trajectory = graph_utils::buildTrajectory(transforms);
  finish = std::chrono::high_resolution_clock::now();
  milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;

  // Compute the pairwise consistency
  std::cout << "Pairwise consistency computation.";
  start = std::chrono::high_resolution_clock::now();
  robust_multirobot_slam::PairwiseConsistency pairwise_consistency(transforms, loop_closure_list, trajectory);
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
  std::cout << "Max clique Size : " << iMaxClique;
  finish = std::chrono::high_resolution_clock::now();
  milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;
  max_clique_data.clear();

  return 0;
}