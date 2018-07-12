#ifndef ROBUST_MULTIROBOT_SLAM_H
#define ROBUST_MULTIROBOT_SLAM_H

#include "graph_utils.h"
#include "geometry_msgs/PoseWithCovariance.h"

#include <eigen3/Eigen/Geometry>

namespace robust_multirobot_slam {

    Eigen::MatrixXi computeConsistentMeasurementsMatrix(const std::map<std::pair<size_t,size_t>, graph_utils::Transform>& transforms,
                                                        const std::list<std::pair<size_t,size_t>>& loop_closure_list,
                                                        const std::map<size_t, graph_utils::TrajectoryPose>& trajectory);
    
    geometry_msgs::PoseWithCovariance computeConsistencyPose(const geometry_msgs::PoseWithCovariance& aXij, 
                                                            const geometry_msgs::PoseWithCovariance& bXlk, 
                                                            const geometry_msgs::PoseWithCovariance& abZik, 
                                                            const geometry_msgs::PoseWithCovariance& abZjl);

    double computeSquaredMahalanobisDistance(const geometry_msgs::PoseWithCovariance& pose);

    geometry_msgs::PoseWithCovariance composeOnTrajectory(const size_t& id1, const size_t& id2, const std::map<size_t, 
                                                            graph_utils::TrajectoryPose>& trajectory);

}

#endif