// author: Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef PAIRWISE_CONSISTENCY_H
#define PAIRWISE_CONSISTENCY_H

#include "graph_utils.h"
#include "geometry_msgs/PoseWithCovariance.h"

#include <eigen3/Eigen/Geometry>

namespace robust_multirobot_slam {

    /**
     * Class for the computation of the pairwise consistency of loop closure edges
     */ 
    class PairwiseConsistency {
      public:
        /**
         * Constructor
         */
        PairwiseConsistency(const std::map<std::pair<size_t,size_t>, graph_utils::Transform>& transforms,
                                        const std::list<std::pair<size_t,size_t>>& loop_closure_list,
                                        const std::map<size_t, graph_utils::TrajectoryPose>& trajectory):
                                        loop_closure_list_(loop_closure_list), transforms_(transforms), trajectory_(trajectory){};

        /**
         * Computation of the consistency matrix
         */ 
        Eigen::MatrixXi computeConsistentMeasurementsMatrix();

      private:

        geometry_msgs::PoseWithCovariance computeConsistencyPose(const geometry_msgs::PoseWithCovariance& aXij, 
                                                        const geometry_msgs::PoseWithCovariance& bXlk, 
                                                        const geometry_msgs::PoseWithCovariance& abZik, 
                                                        const geometry_msgs::PoseWithCovariance& abZjl);

        double computeSquaredMahalanobisDistance(const geometry_msgs::PoseWithCovariance& pose);

        geometry_msgs::PoseWithCovariance composeOnTrajectory(const size_t& id1, const size_t& id2);

        std::map<size_t, graph_utils::TrajectoryPose> trajectory_;

        std::list<std::pair<size_t,size_t>> loop_closure_list_;

        std::map<std::pair<size_t,size_t>, graph_utils::Transform> transforms_;

    };          

}

#endif