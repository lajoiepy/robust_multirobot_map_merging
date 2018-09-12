// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef PAIRWISE_CONSISTENCY_H
#define PAIRWISE_CONSISTENCY_H

#include "graph_utils/graph_utils_functions.h"
#include "geometry_msgs/PoseWithCovariance.h"

#include <eigen3/Eigen/Geometry>

/** \namespace pairwise_consistency
 *  \brief This namespace encapsulates the tools for the pairwise consistency computation.
 */
namespace pairwise_consistency {

    /** \class PairwiseConsistency
     * \brief Class for the computation of the pairwise consistency of loop closure edges
     */ 
    class PairwiseConsistency {
      public:
        /**
         * \brief Constructor
         *
         * @param transforms_robot1 Measurements of robot 1
         * @param transforms_robot2 Measurements of robot 2
         * @param transforms_interrobot Inter-robot measurements
         * @param loop_closures Nodes ID in loop closures
         * @param trajectory_robot1 Precomputed trajectory of robot 1
         * @param trajectory_robot2 Precomputed trajectory of robot 2
         * @param nb_degree_freedom Number of degree of freedom of the robots measurements.
         */
        PairwiseConsistency(const graph_utils::Transforms& transforms_robot1,
                            const graph_utils::Transforms& transforms_robot2,
                            const graph_utils::Transforms& transforms_interrobot,
                            const graph_utils::LoopClosures& loop_closures,
                            const graph_utils::Trajectory& trajectory_robot1,
                            const graph_utils::Trajectory& trajectory_robot2,
                            uint8_t nb_degree_freedom):
                            loop_closures_(loop_closures), transforms_robot1_(transforms_robot1), 
                            transforms_robot2_(transforms_robot2), transforms_interrobot_(transforms_interrobot),
                            trajectory_robot1_(trajectory_robot1), trajectory_robot2_(trajectory_robot2),
                            nb_degree_freedom_(nb_degree_freedom){};

        /**
         * \brief Computation of the consistency matrix
         *
         *
         * @returns the consistency matrix
         */ 
        Eigen::MatrixXi computeConsistentMeasurementsMatrix();

        /*
         * Accessors
         */

        /**
         * \brief Accessor
         *
         * @returns list of loop closures
         */
        const graph_utils::LoopClosures& getLoopClosures() const; 

      private:

        /**
         * \brief Computes the consistency loop : aXij + abZjl + bXlk - abZik (see references)
         *
         * @param aXij
         * @param bXlk
         * @param abZik
         * @param abZjl
         * @return
         */
        geometry_msgs::PoseWithCovariance computeConsistencyPose(const geometry_msgs::PoseWithCovariance& aXij, 
                                                        const geometry_msgs::PoseWithCovariance& bXlk, 
                                                        const geometry_msgs::PoseWithCovariance& abZik, 
                                                        const geometry_msgs::PoseWithCovariance& abZjl);

        /**
         * \brief Compute the Mahalanobis Distance of the input pose (result of pose_a-pose_b)
         *
         * @param transform pose measurement describing the difference between two poses.
         * @returns Mahalanobis Distance
         */
        double computeSquaredMahalanobisDistance(const geometry_msgs::PoseWithCovariance& transform);

        /**
         * \brief This function returns the transform the two specified poses on the desired robot trajectory
         *
         * @param id1 first pose ID
         * @param id2 second pose ID
         * @param robot_id robot ID to select to desired trajectory
         * @return transform (pose measurements) between the two poses
         */
        geometry_msgs::PoseWithCovariance composeOnTrajectory(const size_t& id1, const size_t& id2, const size_t& robot_id);

        graph_utils::LoopClosures loop_closures_;///< loop_closures to consider

        graph_utils::Transforms transforms_robot1_, transforms_robot2_, transforms_interrobot_;///< Measurements for each robot

        graph_utils::Trajectory trajectory_robot1_, trajectory_robot2_;///< Trajectory of the robots

        uint8_t nb_degree_freedom_;///< Number of degree of freedom of the measurements.
    };          

}

#endif