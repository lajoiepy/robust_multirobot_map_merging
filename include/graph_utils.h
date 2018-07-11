#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include "geometry_msgs/PoseWithCovariance.h"

#include <map>
#include <list>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt_bridge/pose.h>

namespace graph_utils {

/** Struct defining a transformation between two poses */
struct Transform {
    size_t i, j;
    geometry_msgs::PoseWithCovariance pose;
    bool is_loop_closure;
};

/** Struct defining a pose in a trav */
struct TrajectoryPose {
    size_t id;
    geometry_msgs::PoseWithCovariance pose;
};

void parseG2ofile(const std::string &filename, size_t &num_poses, 
    std::map<std::pair<size_t,size_t>, graph_utils::Transform>& tranforms,
    std::list<std::pair<size_t,size_t>>& loop_closure_list);

void poseCompose(const geometry_msgs::PoseWithCovariance &a,
                const geometry_msgs::PoseWithCovariance &b,
                geometry_msgs::PoseWithCovariance &out);

void poseInverseCompose(const geometry_msgs::PoseWithCovariance &a,
                const geometry_msgs::PoseWithCovariance &b,
                geometry_msgs::PoseWithCovariance &out);

void poseInverse(const geometry_msgs::PoseWithCovariance &a,
                geometry_msgs::PoseWithCovariance &out);

std::map<size_t, graph_utils::TrajectoryPose> buildTrajectory(const std::map<std::pair<size_t,size_t>, graph_utils::Transform>& transforms);

}

#endif