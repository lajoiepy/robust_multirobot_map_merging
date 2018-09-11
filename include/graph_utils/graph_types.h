// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef GRAPH_UTILS_TYPES_H
#define GRAPH_UTILS_TYPES_H

#include "geometry_msgs/PoseWithCovariance.h"

#include <map>
#include <vector>

/** \namespace graph_utils
 *  \brief This namespace encapsulates utility functions to manipulate graphs
 */
namespace graph_utils {

/** \struct Transform
 *  \brief Structure defining a transformation between two poses
 */
struct Transform {
    size_t i, j;
    geometry_msgs::PoseWithCovariance pose;
    bool is_loop_closure;
};

/** \struct Transforms
 *  \brief Structure defining a std::map of transformations
 */
struct Transforms {
    size_t start_id, end_id;
    std::map<std::pair<size_t,size_t>, graph_utils::Transform> transforms;
};

/** \struct TrajectoryPose
 *  \brief Structure defining a pose in a robot trajectory
 */
struct TrajectoryPose {
    size_t id;
    geometry_msgs::PoseWithCovariance pose;
};

/** \struct Trajectory
 *  \brief Structure defining a robot trajectory
 */
struct Trajectory {
    size_t start_id, end_id;
    std::map<size_t, graph_utils::TrajectoryPose> trajectory_poses;
};

/** \typedef LoopClosures
 *  \brief type to store poses IDs of loop closures.
 */
/** Type defining a list of pair of poses with a loop closure */
typedef std::vector<std::pair<size_t,size_t>> LoopClosures;

}
#endif