// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef GLOBAL_MAP_SOLVER_H
#define GLOBAL_MAP_SOLVER_H

#include "robot_local_map/robot_local_map.h"
#include "pairwise_consistency/pairwise_consistency.h"
#include "SESync/SESync.h"
#include "SESync/SESync_utils.h"
#include <string>

namespace global_map_solver {
    /** \class GlobalMapSolver
     * \brief Class computing the global map from multiple robots local maps.
     */ 
    class GlobalMapSolver {
      public:
        /** \var CONSISTENCY_MATRIX_FILE_NAME
         * \brief File name in which the consistency matrix will be saved
         */
        static const std::string CONSISTENCY_MATRIX_FILE_NAME;

        /** \var CONSISTENCY_LOOP_CLOSURES_FILE_NAME
         * \brief File name in which the consistent loop closures will be saved
         */
        static const std::string CONSISTENCY_LOOP_CLOSURES_FILE_NAME;

        /**
         * \brief Constructor
         *
         * @param robot1_local_map Local map of robot 1.
         * @param robot2_local_map Local map of robot 2.
         * @param interrobot_measurements Inter-robot measurements.
         */
        GlobalMapSolver(const robot_local_map::RobotLocalMap& robot1_local_map,
                        const robot_local_map::RobotLocalMap& robot2_local_map,
                        const robot_local_map::RobotMeasurements& interrobot_measurements);

        /**
         * \brief Function that solves the global maps according to the current constraints
         *
         * @return the size of the maximum clique.
         */
        int solveGlobalMap();

      private:
        pairwise_consistency::PairwiseConsistency pairwise_consistency_; ///< Pairwise consistency solver.

        /**
         * \brief This functions fill the structure measurements_t whitout the spurious measurements
         * in order to use SE-Sync.
         *
         * @param loop_closures List of loop closures
         * @param max_clique_data List of valid loop closures ID
         * @return the formatted measurements
         */
        SESync::measurements_t fillMeasurements(const std::vector<int>& max_clique_data);

    }; 
}

#endif