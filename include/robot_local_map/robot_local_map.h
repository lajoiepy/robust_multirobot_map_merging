// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef ROBOT_LOCAL_MAP_H
#define ROBOT_LOCAL_MAP_H

#include "graph_utils/graph_types.h"

namespace robot_local_map {
    /**
     * Class that contains the local map of a single robot.
     */ 
    class RobotLocalMap {
      public:
        /**
         * Constructor
         */
        RobotLocalMap(const std::string & file_name);

        /**
         * Accessors
         */
        const graph_utils::TransformMap& getTransforms() const;
        const size_t& getNumPoses() const;
        const graph_utils::LoopClosures& getLoopClosures() const;
        const graph_utils::Trajectory& getTrajectory() const;
        const uint8_t& getNbDegreeFreedom() const;

      private:
        graph_utils::TransformMap transforms_;
        size_t num_poses_;
        graph_utils::LoopClosures loop_closures_;
        graph_utils::Trajectory trajectory_;
        uint8_t nb_degree_freedom_;
    };          
}

#endif