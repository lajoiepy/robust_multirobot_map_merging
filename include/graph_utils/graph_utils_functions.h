// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef GRAPH_UTILS_FUNCTIONS_H
#define GRAPH_UTILS_FUNCTIONS_H

#include "graph_utils/graph_types.h"
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt_bridge/pose.h>
#include <iostream>
#include <fstream>

namespace graph_utils {
/**
 * This function parse .g2o files.
 * The specification of this format is available here : https://github.com/RainerKuemmerle/g2o/wiki/File-Format
 */
void parseG2ofile(const std::string &filename, size_t &num_poses, 
    TransformMap& tranform_map,
    LoopClosures& loop_closures, 
    const bool& only_loop_closures);

/**
 * This function combine to geometric poses with covariance.
 * out = a + b
 * Uses the mrpt library : https://www.mrpt.org/
 */
void poseCompose(const geometry_msgs::PoseWithCovariance &a,
                const geometry_msgs::PoseWithCovariance &b,
                geometry_msgs::PoseWithCovariance &out);

/**
 * This function combine to geometric poses with covariance.
 * out = a - b
 * Uses the mrpt library : https://www.mrpt.org/
 */
void poseInverseCompose(const geometry_msgs::PoseWithCovariance &a,
                const geometry_msgs::PoseWithCovariance &b,
                geometry_msgs::PoseWithCovariance &out);

/**
 * This function invert a geometric pose with covariance.
 * out = I - a
 * Uses the mrpt library : https://www.mrpt.org/
 */
void poseInverse(const geometry_msgs::PoseWithCovariance &a,
                geometry_msgs::PoseWithCovariance &out);

/**
 * This function precompute the trajectory by composing the sucessive poses.
 */ 
Trajectory buildTrajectory(const TransformMap& transform_map);

/**
 * This function prints the consistency matrix to the format expected by the maximum clique solver
 * Fast Max-Cliquer (http://cucis.ece.northwestern.edu/projects/MAXCLIQUE/) 
 */ 
void printConsistencyGraph(const Eigen::MatrixXi& consistency_matrix, const std::string& file_name);

/**
 * This function check if a pose is include in a trajectory.
 */
bool isInTrajectory(const Trajectory& trajectory, const size_t& pose_id);

/**
 * This function prints a list of consistent loop closures.
 */
void printConsistentLoopClosures(const LoopClosures& loop_closures, const std::vector<int>& max_clique_data, const std::string& file_name);
}

#endif