#include "graph_utils.h"

#include <fstream>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Geometry>

namespace graph_utils {

void parseG2ofile(const std::string &filename, size_t &num_poses, 
    std::map<std::pair<size_t,size_t>, graph_utils::Transform>& transforms,
    std::list<std::pair<size_t,size_t>>& loop_closure_list) {

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

  // Open the file for reading
  std::ifstream infile(filename);

  num_poses = 0;

  while (std::getline(infile, line)) {
    // Construct a stream from the string
    std::stringstream strstrm(line);

    // Extract the first token from the string
    strstrm >> token;

    if (token == "EDGE_SE2") {
      // This is a 2D pose measurement

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
    
    if (max_pair <= num_poses) {
      transform.is_loop_closure = true;
      loop_closure_list.emplace_back(std::make_pair(i,j));
    } else {
      num_poses = max_pair;
    }

    transforms.emplace(std::make_pair(std::make_pair(i,j), transform));
  } // while

  infile.close();

  num_poses++; // Account for the use of zero-based indexing
}

}