// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "pairwise_consistency/pairwise_consistency.h"

namespace pairwise_consistency {

Eigen::MatrixXi PairwiseConsistency::computeConsistentMeasurementsMatrix() {
    // Determination of the chi squared threshold (numbers from chi-squared table)
    double threshold;
    if (nb_degree_freedom_ == 3){
        threshold = 0.58;
    } else {
        threshold = 2.20;
    }

    // Preallocate consistency matrix
    Eigen::MatrixXi consistency_matrix(loop_closures_.size(), loop_closures_.size());

    // Iterate on loop closures
    size_t u = 0;
    for (const auto& loop_closure_1: loop_closures_) {
        size_t v = 0;
        for (const auto& loop_closure_2: loop_closures_) {
            if (u < v) {
                // Extract pose indexes
                size_t i,j,k,l;
                i = loop_closure_1.first;
                k = loop_closure_1.second;
                j = loop_closure_2.first;
                l = loop_closure_2.second;

                // Check if the loop closures are interrobot. 
                // It is the case if {i,j} are elements of trajectory_robot1 and {k,l} are elements of trajectory_robot2.
                // Or the inverse.
                bool is_config_r12 = graph_utils::isInTrajectory(trajectory_robot1_, i) && graph_utils::isInTrajectory(trajectory_robot1_, j) &&
                    graph_utils::isInTrajectory(trajectory_robot2_, k) && graph_utils::isInTrajectory(trajectory_robot2_, l);
                bool is_config_r21 = graph_utils::isInTrajectory(trajectory_robot2_, i) && graph_utils::isInTrajectory(trajectory_robot2_, j) &&
                    graph_utils::isInTrajectory(trajectory_robot1_, k) && graph_utils::isInTrajectory(trajectory_robot1_, l);
                // Compute only if they are interrobot loop closures
                if (is_config_r12 || is_config_r21) {
                    // Extract transforms
                    geometry_msgs::PoseWithCovariance abZik = (*transforms_interrobot_.transforms.find(loop_closure_1)).second.pose;
                    geometry_msgs::PoseWithCovariance abZjl = (*transforms_interrobot_.transforms.find(loop_closure_2)).second.pose; 
                    geometry_msgs::PoseWithCovariance aXij, bXlk;
                    if (is_config_r12) {
                        aXij = composeOnTrajectory(i, j, 1);  
                        bXlk = composeOnTrajectory(l, k, 2); 
                    } else {
                        aXij = composeOnTrajectory(i, j, 2);  
                        bXlk = composeOnTrajectory(l, k, 1); 
                    }
                    // Compute the consistency pose (should be near Identity if consistent)
                    geometry_msgs::PoseWithCovariance consistency_pose = computeConsistencyPose(aXij, bXlk, abZik, abZjl);
                    // Compute the Mahalanobis distance
                    double distance = computeSquaredMahalanobisDistance(consistency_pose);
                    // Apply threshold on the chi-squared distribution
                    if (distance < threshold) {
                        consistency_matrix(u,v) = 1;
                    } else {
                        consistency_matrix(u,v) = 0;
                    }
                }
            }
            v++;
        }
        u++;
    }
    return consistency_matrix;
}

geometry_msgs::PoseWithCovariance PairwiseConsistency::computeConsistencyPose(const geometry_msgs::PoseWithCovariance& aXij, 
                                                        const geometry_msgs::PoseWithCovariance& bXlk, 
                                                        const geometry_msgs::PoseWithCovariance& abZik, 
                                                        const geometry_msgs::PoseWithCovariance& abZjl) {
    // Consistency loop : aXij + abZjl + bXlk - abZik
    geometry_msgs::PoseWithCovariance out1, out2, result;
    graph_utils::poseCompose(aXij, abZjl, out1);
    graph_utils::poseCompose(out1, bXlk, out2);
    graph_utils::poseInverseCompose(out2, abZik, result);

    return result;
}

double PairwiseConsistency::computeSquaredMahalanobisDistance(const geometry_msgs::PoseWithCovariance& transform) {
    // Extraction of the covariance matrix
    Eigen::Matrix<double, 6, 6> covariance_matrix;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            covariance_matrix(i,j) = transform.covariance[i*6+j];
        }
    }

    // Extraction of the pose vector
    Eigen::Matrix<double, 6, 1> pose_vector(6);
    pose_vector(0) = transform.pose.position.x;
    pose_vector(1) = transform.pose.position.y;
    pose_vector(2) = transform.pose.position.z;
    pose_vector(3) = transform.pose.orientation.x;
    pose_vector(4) = transform.pose.orientation.y;
    pose_vector(5) = transform.pose.orientation.z;

    // Computation of the squared Mahalanobis distance
    double distance = pose_vector.transpose() * covariance_matrix * pose_vector;
    return distance;
}

geometry_msgs::PoseWithCovariance PairwiseConsistency::composeOnTrajectory(const size_t& id1, const size_t& id2, const size_t& robot_id) {
    // Select trajectory
    graph_utils::Trajectory trajectory;
    if (robot_id == 1) {
        trajectory = trajectory_robot1_;
    } else {
        trajectory = trajectory_robot2_;
    }
    
    // Extraction of the poses on the trajectory
    graph_utils::TrajectoryPose pose1 = (*trajectory.trajectory_poses.find(id1)).second;
    graph_utils::TrajectoryPose pose2 = (*trajectory.trajectory_poses.find(id2)).second;
    // Computation of the transformation
    geometry_msgs::PoseWithCovariance result;
    graph_utils::poseInverseCompose(pose2.pose, pose1.pose, result);
    return result;
}

const graph_utils::LoopClosures& PairwiseConsistency::getLoopClosures() const {
    return loop_closures_;
}

const graph_utils::Transforms& PairwiseConsistency::getTransformsRobot1() const{
    return transforms_robot1_;
}

const graph_utils::Transforms& PairwiseConsistency::getTransformsRobot2() const{
    return transforms_robot2_;
}

const graph_utils::Transforms& PairwiseConsistency::getTransformsInterRobot() const{
    return transforms_interrobot_;
}

}