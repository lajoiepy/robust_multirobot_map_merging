// author: Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include "geometry_msgs/PoseWithCovariance.h"

#include <map>
#include <list>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt_bridge/pose.h>
#include <iostream>
#include <fstream>

namespace graph_utils {

/** Struct defining a transformation between two poses */
struct Transform {
    size_t i, j;
    geometry_msgs::PoseWithCovariance pose;
    bool is_loop_closure;
};

/** Struct defining a pose in a trajectory */
struct TrajectoryPose {
    size_t id;
    geometry_msgs::PoseWithCovariance pose;
};

/**
 * This function parse .g2o files.
 * The specification of this format is available here : https://github.com/RainerKuemmerle/g2o/wiki/File-Format
 */
void parseG2ofile(const std::string &filename, size_t &num_poses, 
    std::map<std::pair<size_t,size_t>, graph_utils::Transform>& tranforms,
    std::list<std::pair<size_t,size_t>>& loop_closure_list);

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
std::map<size_t, graph_utils::TrajectoryPose> buildTrajectory(const std::map<std::pair<size_t,size_t>, graph_utils::Transform>& transforms);

/**
 * This function prints the consistency matrix to the format expected by the maximum clique solver
 * Fast Max-Cliquer (http://cucis.ece.northwestern.edu/projects/MAXCLIQUE/) 
 */ 
void printConsistencyGraph(const Eigen::MatrixXi& consistency_matrix);

}

#endif