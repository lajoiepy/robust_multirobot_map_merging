// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "graph_utils/graph_utils_functions.h"

#include <fstream>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Geometry>

using namespace mrpt::poses;
using namespace mrpt::math;

namespace graph_utils {

uint8_t parseG2ofile(const std::string &file_name, size_t &num_poses, 
    Transforms& transforms,
    LoopClosures& loop_closures, 
    const bool& only_loop_closures) {

  // A single pose that will be filled
  graph_utils::Transform transform;
  transform.is_loop_closure = false;

  // A string used to contain the contents of a single line
  std::string line;

  // A string used to extract tokens from each line one-by-one
  std::string token;

  // Preallocate various useful quantities
  double dx, dy, dz, dtheta, dqx, dqy, dqz, dqw, I11, I12, I13, I14, I15, I16,
      I22, I23, I24, I25, I26, I33, I34, I35, I36, I44, I45, I46, I55, I56, I66;

  size_t i, j;

  uint8_t nb_degrees_freedom = -1;

  // Open the file for reading
  std::ifstream infile(file_name);

  if (!infile.is_open()) {
    std::cerr << "Error while opening the file" << std::endl;
    std::abort();
  }

  num_poses = 0;
  bool is_first_iteration = true;
  while (std::getline(infile, line)) {
    // Construct a stream from the string
    std::stringstream strstrm(line);

    // Extract the first token from the string
    strstrm >> token;

    if (token == "EDGE_SE2") {
      // This is a 2D pose measurement
      nb_degrees_freedom = 3;

      // Extract formatted output
      strstrm >> i >> j >> dx >> dy >> dtheta >> I11 >> I12 >> I13 >> I22 >>
          I23 >> I33;

      // Fill in elements of this measurement

      // Pose ids
      transform.i = i;
      transform.j = j;

      // Raw measurements
      transform.pose.pose.position.x = dx;
      transform.pose.pose.position.y = dy;
      transform.pose.pose.position.z = 0;
      transform.pose.pose.orientation.x = 0;
      transform.pose.pose.orientation.y = 0;
      transform.pose.pose.orientation.z = sin(dtheta/2);
      transform.pose.pose.orientation.w = cos(dtheta/2);

      // Covariance
      Eigen::Matrix3d infomation_matrix, covariance_matrix;
      infomation_matrix << I11, I12, I13, I12, I22, I23, I13, I23, I33;
      covariance_matrix = infomation_matrix.inverse();
      transform.pose.covariance[0] = covariance_matrix(0, 0);
      transform.pose.covariance[1] = covariance_matrix(0, 1);
      transform.pose.covariance[5] = covariance_matrix(0, 2);
      transform.pose.covariance[6] = covariance_matrix(1, 0);
      transform.pose.covariance[7] = covariance_matrix(1, 1);
      transform.pose.covariance[11] = covariance_matrix(1, 2);
      transform.pose.covariance[30] = covariance_matrix(2, 0);
      transform.pose.covariance[31] = covariance_matrix(2, 1);
      transform.pose.covariance[35] = covariance_matrix(2, 2);

    } else if (token == "EDGE_SE3:QUAT") {
      // This is a 3D pose measurement
      nb_degrees_freedom = 6;

      // Extract formatted output
      strstrm >> i >> j >> dx >> dy >> dz >> dqx >> dqy >> dqz >> dqw >> I11 >>
          I12 >> I13 >> I14 >> I15 >> I16 >> I22 >> I23 >> I24 >> I25 >> I26 >>
          I33 >> I34 >> I35 >> I36 >> I44 >> I45 >> I46 >> I55 >> I56 >> I66;

      // Fill in elements of the measurement

      // Pose ids
      transform.i = i;
      transform.j = j;

      // Raw measurements
      transform.pose.pose.position.x = dx;
      transform.pose.pose.position.y = dy;
      transform.pose.pose.position.z = dz;
      transform.pose.pose.orientation.x = dqx;
      transform.pose.pose.orientation.y = dqy;
      transform.pose.pose.orientation.z = dqz;
      transform.pose.pose.orientation.w = dqw;

      // Covariance
      Eigen::Matrix<double, 6, 6> infomation_matrix, covariance_matrix;
      infomation_matrix <<  I11, I12, I13, I14, I15, I16,
                            I12, I22, I23, I24, I25, I26,
                            I13, I23, I33, I34, I35, I36,
                            I14, I24, I34, I44, I45, I46,
                            I15, I25, I35, I45, I55, I56,
                            I16, I26, I36, I46, I56, I66;
      covariance_matrix = infomation_matrix.inverse();

      for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
              transform.pose.covariance[i*6+j] = covariance_matrix(i,j);
          }
      }

    } else if ((token == "VERTEX_SE2") || (token == "VERTEX_SE3:QUAT")) {
      // This is just initialization information, so do nothing
      continue;
    } else {
      std::cout << "Error: unrecognized type: " << token << "!" << std::endl;
      assert(false);
    }

    // Update maximum value of poses found so far
    size_t max_pair = std::max<double>(transform.i, transform.j);
    
    if (only_loop_closures || max_pair <= num_poses) {
      transform.is_loop_closure = true;
      loop_closures.emplace_back(std::make_pair(i,j));
    } else {
      num_poses = max_pair;
      transforms.end_id = j;
    }

    if (is_first_iteration) {
      transforms.start_id = i;
      is_first_iteration = false;
    }

    transforms.transforms.emplace(std::make_pair(std::make_pair(i,j), transform));
  }

  infile.close();

  num_poses++; 
  return nb_degrees_freedom;
}

void poseCompose(const geometry_msgs::PoseWithCovariance &a,
                const geometry_msgs::PoseWithCovariance &b,
                geometry_msgs::PoseWithCovariance &out) {
  CPose3DPDFGaussian A(UNINITIALIZED_POSE), B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  const CPose3DPDFGaussian OUT = A + B;
  mrpt_bridge::convert(OUT, out);
}

void poseInverse(const geometry_msgs::PoseWithCovariance &a,
                geometry_msgs::PoseWithCovariance &out) {
  CPose3DPDFGaussian A(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);

  CPose3DPDFGaussian OUT;
  A.inverse(OUT);
  mrpt_bridge::convert(OUT, out);
}

void poseInverseCompose(const geometry_msgs::PoseWithCovariance &a,
                                  const geometry_msgs::PoseWithCovariance &b,
                                  geometry_msgs::PoseWithCovariance &out) {
  CPose3DPDFGaussian A(UNINITIALIZED_POSE), B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  const CPose3DPDFGaussian OUT = A - B;
  mrpt_bridge::convert(OUT, out);
}

Trajectory buildTrajectory(const Transforms& transforms) {
    // Initialization
    Trajectory trajectory;
    trajectory.start_id = transforms.start_id;
    trajectory.end_id = transforms.end_id;
    size_t current_pose_id = trajectory.start_id;
    geometry_msgs::PoseWithCovariance temp_pose, total_pose;

    // Add first pose at the origin
    graph_utils::TrajectoryPose current_pose;
    current_pose.id = current_pose_id;
    current_pose.pose.pose.orientation.w = 1;
    temp_pose = current_pose.pose;
    trajectory.trajectory_poses.insert(std::make_pair(current_pose_id, current_pose));

    // Initialization
    std::pair<size_t, size_t> temp_pair = std::make_pair(current_pose_id, current_pose_id + 1);
    auto temp_it = transforms.transforms.find(temp_pair);

    // Compositions in chain on the trajectory transforms.
    while (temp_it != transforms.transforms.end() && !(*temp_it).second.is_loop_closure) {
        graph_utils::poseCompose(temp_pose, (*temp_it).second.pose, total_pose);             
        temp_pose = total_pose;
        current_pose_id++;
        current_pose.id = current_pose_id;
        current_pose.pose = total_pose;
        trajectory.trajectory_poses.insert(std::make_pair(current_pose_id, current_pose));
        temp_pair = std::make_pair(current_pose_id, current_pose_id + 1);
        temp_it = transforms.transforms.find(temp_pair);
    }

    return trajectory;
}

void printConsistencyGraph(const Eigen::MatrixXi& consistency_matrix, const std::string& file_name) {
    // Intialization
    int nb_consistent_measurements = 0;
    
    // Format edges.
    std::stringstream ss;
    for (int i = 0; i < consistency_matrix.rows(); i++) {
      for (int j = i; j < consistency_matrix.cols(); j++) {
        if (consistency_matrix(i,j) == 1) {
          ss << i+1 << " " << j+1 << std::endl;
          nb_consistent_measurements++;
        }
      }
    }
    
    // Write to file
    std::ofstream output_file;
    output_file.open(file_name);
    output_file << "%%MatrixMarket matrix coordinate pattern symmetric" << std::endl;
    output_file << consistency_matrix.rows() << " " << consistency_matrix.cols() << " " << nb_consistent_measurements << std::endl;
    output_file << ss.str();
    output_file.close();
}

bool isInTrajectory(const Trajectory& trajectory, const size_t& pose_id) {
  return trajectory.trajectory_poses.find(pose_id) != trajectory.trajectory_poses.end();
}

void printConsistentLoopClosures(const LoopClosures& loop_closures, const std::vector<int>& max_clique_data, const std::string& file_name){
  std::ofstream output_file;
  output_file.open(file_name);
  for (auto loop_closure_id: max_clique_data) {
    // -1 because fast max-clique finder is one-based.
    output_file << loop_closures[loop_closure_id-1].first << " " << loop_closures[loop_closure_id-1].second << std::endl;
  }
  output_file.close();
}

SESync::RelativePoseMeasurement convertTransformToRelativePoseMeasurement(const Transform& t) {
    // A single measurement, whose values we will fill in
    SESync::RelativePoseMeasurement measurement;

    // Pose ids
    measurement.i = t.i;
    measurement.j = t.j;

    // Raw measurements
    measurement.t = Eigen::Vector2d(t.pose.pose.position.x, t.pose.pose.position.y);

    double dtheta = 2*asin(t.pose.pose.orientation.z);
    measurement.R = Eigen::Rotation2Dd(dtheta).toRotationMatrix();

    Eigen::Matrix2d TranCov;
    //TranCov << t.pose.covariance[0], t.pose.covariance[1], t.pose.covariance[1], t.pose.covariance[7];
    measurement.tau = 0.1;//2 / TranCov.trace();

    if (t.pose.covariance[35] != 0) {
        measurement.kappa = 0.1;//1 / t.pose.covariance[35];
    } else {
        std::cerr << "Covariance on rotation null (leads to a division by zero)" << std::endl;
        measurement.kappa = 1000;
    }
    return measurement;
}

}