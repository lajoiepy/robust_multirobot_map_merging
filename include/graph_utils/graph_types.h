// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef GRAPH_UTILS_TYPES_H
#define GRAPH_UTILS_TYPES_H

#include "geometry_msgs/PoseWithCovariance.h"

#include <map>
#include <vector>

namespace graph_utils {

/** Struct defining a transformation between two poses */
struct Transform {
    size_t i, j;
    geometry_msgs::PoseWithCovariance pose;
    bool is_loop_closure;
};

/** Struct defining a map of transformation */
struct TransformMap {
    size_t start_id, end_id;
    std::map<std::pair<size_t,size_t>, graph_utils::Transform> transforms;
};

/** Struct defining a pose in a trajectory */
struct TrajectoryPose {
    size_t id;
    geometry_msgs::PoseWithCovariance pose;
};

/** Struct defining a trajectory */
struct Trajectory {
    size_t start_id, end_id;
    std::map<size_t, graph_utils::TrajectoryPose> trajectory_poses;
};

/** Type defining a list of pair of poses with a loop closure */
typedef std::vector<std::pair<size_t,size_t>> LoopClosures;

}
#endif