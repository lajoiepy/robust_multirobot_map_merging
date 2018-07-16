// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "robot_local_map/robot_local_map.h"
#include "graph_utils/graph_utils_functions.h"

namespace robot_local_map {

RobotLocalMap::RobotLocalMap(const std::string & file_name){
    nb_degree_freedom_ = graph_utils::parseG2ofile(file_name, num_poses_, transforms_, loop_closures_, false);
    trajectory_ = graph_utils::buildTrajectory(transforms_);
}

const graph_utils::TransformMap& RobotLocalMap::getTransforms() const {
    return transforms_;
}

const size_t& RobotLocalMap::getNumPoses() const {
    return num_poses_;
}

const graph_utils::LoopClosures& RobotLocalMap::getLoopClosures() const {
    return loop_closures_;
}

const graph_utils::Trajectory& RobotLocalMap::getTrajectory() const {
    return trajectory_;
}

const uint8_t& RobotLocalMap::getNbDegreeFreedom() const {
    return nb_degree_freedom_;
}

}