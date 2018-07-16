// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef ROBOT_MEASUREMENTS_H
#define ROBOT_MEASUREMENTS_H

#include "graph_utils/graph_types.h"

namespace robot_local_map {
    /**
     * Class that contains the local map of a single robot.
     */ 
    class RobotMeasurements {
      public:
        /**
         * Constructor
         */
        RobotMeasurements(const std::string & file_name, const bool& is_only_loop_closures);

        /**
         * Accessors
         */
        virtual const graph_utils::TransformMap& getTransforms() const;
        virtual const size_t& getNumPoses() const;
        virtual const graph_utils::LoopClosures& getLoopClosures() const;
        virtual const uint8_t& getNbDegreeFreedom() const;

      protected:
        graph_utils::TransformMap transforms_;
        size_t num_poses_;
        graph_utils::LoopClosures loop_closures_;
        uint8_t nb_degree_freedom_;
    };          
}

#endif