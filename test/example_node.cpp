// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "graph_utils/graph_utils_functions.h"
#include "pairwise_consistency/pairwise_consistency.h"
#include "robot_local_map/robot_local_map.h"
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
  size_t num_poses_interrobot;
  graph_utils::TransformMap transforms_interrobot;
  graph_utils::LoopClosures loop_closures;

  // Parse the graph files
  std::cout << "Parsing of the following files : " << robot1_file_name << ", " << robot2_file_name << ", " << interrobot_file_name;
  auto start = std::chrono::high_resolution_clock::now();
  auto interrobot_measurements = robot_local_map::RobotMeasurements(interrobot_file_name, true);
  auto finish = std::chrono::high_resolution_clock::now();
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;
  
  // Compute the non-optimized trajectory
  std::cout << "Trajectory computation." ;
  start = std::chrono::high_resolution_clock::now();
  auto robot1_local_map = robot_local_map::RobotLocalMap(robot1_file_name);
  auto robot2_local_map = robot_local_map::RobotLocalMap(robot2_file_name);
  finish = std::chrono::high_resolution_clock::now();
  milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;

  // Compute the pairwise consistency
  std::cout << "Pairwise consistency computation.";
  start = std::chrono::high_resolution_clock::now();
  pairwise_consistency::PairwiseConsistency pairwise_consistency(robot1_local_map.getTransforms(), robot2_local_map.getTransforms(), interrobot_measurements.getTransforms(), interrobot_measurements.getLoopClosures(), robot1_local_map.getTrajectory(), robot2_local_map.getTrajectory());
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
  FMC::CGraphIO gio;
  gio.readGraph(CONSISTENCY_MATRIX_FILE_NAME);
  int iMaxClique = 0;
  std::vector<int> max_clique_data;
  iMaxClique = FMC::maxClique(gio, iMaxClique, max_clique_data);
  std::cout << " - Max clique Size : " << iMaxClique;
  finish = std::chrono::high_resolution_clock::now();
  milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;
  
  // Reassign result and print consistent loop closures in output file
  std::cout << "Print result in " << output_file_name;
  start = std::chrono::high_resolution_clock::now();
  graph_utils::printConsistentLoopClosures(loop_closures, max_clique_data, output_file_name);
  finish = std::chrono::high_resolution_clock::now();
  milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
  std::cout << " | Completed (" << milliseconds.count() << "ms)" << std::endl;

  // Clean up
  max_clique_data.clear();

  return 0;
}