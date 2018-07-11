#ifndef ROBUST_MULTIROBOT_SLAM_H
#define ROBUST_MULTIROBOT_SLAM_H

#include "graph_utils.h"
#include "geometry_msgs/PoseWithCovariance.h"

#include <eigen3/Eigen/Geometry>

namespace robust_multirobot_slam {

    Eigen::MatrixXd computeConsistentMeasurementsMatrix(const std::map<std::pair<size_t,size_t>, graph_utils::Transform>& transforms);
    
    geometry_msgs::PoseWithCovariance computeConsistencyPose(const geometry_msgs::PoseWithCovariance& aXij, 
                                                            const geometry_msgs::PoseWithCovariance& bXlk, 
                                                            const geometry_msgs::PoseWithCovariance& abZik, 
                                                            const geometry_msgs::PoseWithCovariance& abZjl);

    double computeSquaredMahalanobisDistance(const geometry_msgs::PoseWithCovariance& pose);

}

#endif