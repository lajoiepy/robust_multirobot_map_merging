#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include "geometry_msgs/PoseWithCovariance.h"

#include <map>
#include <list>

namespace graph_utils {

/** Struct defining a transformation between two poses */
struct Transform {
    size_t i, j;
    geometry_msgs::PoseWithCovariance pose;
    bool is_loop_closure;
};

void parse_g2o_file(const std::string &filename, size_t &num_poses, 
    std::map<std::pair<size_t,size_t>, graph_utils::Transform>& tranforms,
    std::list<std::pair<size_t,size_t>>& loop_closure_list);

}

#endif