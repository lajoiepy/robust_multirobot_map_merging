// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef GLOBAL_MAP_SOLVER_H
#define GLOBAL_MAP_SOLVER_H

#include "robot_local_map/robot_local_map.h"
#include "pairwise_consistency/pairwise_consistency.h"
#include <string>

namespace global_map_solver {
    /**
     * Class that computes the global map of multiple robots.
     */ 
    class GlobalMapSolver {
      public:
        /**
         * File name in which the consistency matrix will be saved
         */
        static const std::string CONSISTENCY_MATRIX_FILE_NAME;

        /**
         * File name in which the consistent loop closures will be saved
         */
        static const std::string CONSISTENCY_LOOP_CLOSURES_FILE_NAME;

        /**
         * Constructor
         */
        GlobalMapSolver(const robot_local_map::RobotLocalMap& robot1_local_map,
                        const robot_local_map::RobotLocalMap& robot2_local_map,
                        const robot_local_map::RobotMeasurements& interrobot_measurements);

        /**
         * Functions that solves the global maps according to the current constraints
         * returns the size of the maximum clique.
         */
        int solveGlobalMap();

      private:
        pairwise_consistency::PairwiseConsistency pairwise_consistency_;

    }; 
}

#endif