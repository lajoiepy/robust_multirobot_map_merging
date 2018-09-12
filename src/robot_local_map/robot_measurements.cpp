// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "robot_local_map/robot_measurements.h"
#include "graph_utils/graph_utils_functions.h"

namespace robot_local_map {

RobotMeasurements::RobotMeasurements(const std::string & file_name, const bool& is_only_loop_closures){
    nb_degree_freedom_ = graph_utils::parseG2ofile(file_name, num_poses_, transforms_, loop_closures_, is_only_loop_closures);
}

const graph_utils::Transforms& RobotMeasurements::getTransforms() const {
    return transforms_;
}

const size_t& RobotMeasurements::getNumPoses() const {
    return num_poses_;
}

const graph_utils::LoopClosures& RobotMeasurements::getLoopClosures() const {
    return loop_closures_;
}

const uint8_t& RobotMeasurements::getNbDegreeFreedom() const {
    return nb_degree_freedom_;
}

}