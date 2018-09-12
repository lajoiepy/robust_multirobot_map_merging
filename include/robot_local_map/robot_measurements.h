// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef ROBOT_MEASUREMENTS_H
#define ROBOT_MEASUREMENTS_H

#include "graph_utils/graph_types.h"

/** \namespace robot_local_map
 *  \brief This namespace encapsulates classes and functions related to the local maps of the individual robots.
 */ 
namespace robot_local_map {

    /** \class RobotMeasurements
     *  \brief Class that contains the measurements of a single robot.
     * 
     *  The measurements are in the TransformMap and LoopClosures contains the nodes IDs f the loop closures.
     */ 
    class RobotMeasurements {
      public:
        /**
         * \brief Constructor
         * @param file_name
         */
        RobotMeasurements(const std::string & file_name, const bool& is_only_loop_closures);

        /*
         * Accessors
         */

        /** \brief Accessor
         * @return the robot measurements
         */
        virtual const graph_utils::Transforms& getTransforms() const;

        /** \brief Accessor
         * @return the number of poses in the robot local map
         */
        virtual const size_t& getNumPoses() const;

        /** \brief Accessor
         * @return the loop closures in the robot local map
         */
        virtual const graph_utils::LoopClosures& getLoopClosures() const;

        /** \brief Accessor
         * @return the number of degree of freedom of the robot local map
         */
        virtual const uint8_t& getNbDegreeFreedom() const;

      protected:
        graph_utils::Transforms transforms_; ///< std::map containing all the local measurements of the robot.
        size_t num_poses_; ///< Number of poses in the map
        graph_utils::LoopClosures loop_closures_; ///< std::vector containing the ID pairs of the loop closures
        uint8_t nb_degree_freedom_; ///< Number of degree of freedom of the measurements, typically 3 in 2D and 6 in 3D.
    };          
}

#endif