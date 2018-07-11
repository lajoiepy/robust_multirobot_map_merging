#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include "geometry_msgs/PoseWithCovariance.h"

#include <vector>

namespace graph_utils {

/** Struct defining a transformation between two poses */
struct Transform {
    size_t i, j;
    geometry_msgs::PoseWithCovariance pose;
    bool is_interrobot;
};

std::vector<Transform> parse_g2o_file(const std::string &filename, size_t &num_poses);

}

#endif