// author: Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "pairwise_consistency.h"
#include "pose_cov_ops/pose_cov_ops.h"
#define THRESHOLD 1.635

namespace robust_multirobot_slam {

    Eigen::MatrixXi PairwiseConsistency::computeConsistentMeasurementsMatrix() {
        // Preallocate consistency matrix
        Eigen::MatrixXi consistency_matrix(loop_closure_list_.size(), loop_closure_list_.size());

        // Iterate on loop closures
        size_t u = 0;
        for (std::list<std::pair<size_t,size_t>>::const_iterator it_row = loop_closure_list_.begin(); it_row != loop_closure_list_.end(); ++it_row) {
            size_t v = 0;
            for (std::list<std::pair<size_t,size_t>>::const_iterator it_col = loop_closure_list_.begin(); it_col != loop_closure_list_.end(); ++it_col) {
                if (u != v) {
                    // Extract pose indexes
                    size_t i,j,k,l;
                    i = (*it_row).first;
                    k = (*it_row).second;
                    j = (*it_col).first;
                    l = (*it_col).second;
                    // Extract transforms
                    geometry_msgs::PoseWithCovariance abZik = (*transforms_.find(*it_row)).second.pose;
                    geometry_msgs::PoseWithCovariance abZjl = (*transforms_.find(*it_col)).second.pose; 
                    geometry_msgs::PoseWithCovariance aXij = composeOnTrajectory(i, j);  
                    geometry_msgs::PoseWithCovariance bXlk = composeOnTrajectory(l, k); 
                    // Compute the consistency pose (should be near Identity if consistent)
                    geometry_msgs::PoseWithCovariance consistency_pose = computeConsistencyPose(aXij, bXlk, abZik, abZjl);
                    // Compute the Mahalanobis distance
                    double distance = computeSquaredMahalanobisDistance(consistency_pose);
                    // Apply threshold on the chi-squared distribution
                    if (distance < THRESHOLD) {
                        consistency_matrix(u,v) = 1;
                    } else {
                        consistency_matrix(u,v) = 0;
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

    double PairwiseConsistency::computeSquaredMahalanobisDistance(const geometry_msgs::PoseWithCovariance& pose) {
        // Extraction of the covariance matrix
        Eigen::Matrix<double, 6, 6> covariance_matrix;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                covariance_matrix(i,j) = pose.covariance[i*6+j];
            }
        }

        // Extraction of the pose vector
        Eigen::Matrix<double, 6, 1> pose_vector(6);
        pose_vector(0) = pose.pose.position.x;
        pose_vector(1) = pose.pose.position.y;
        pose_vector(2) = pose.pose.position.z;
        pose_vector(3) = pose.pose.orientation.x;
        pose_vector(4) = pose.pose.orientation.y;
        pose_vector(5) = pose.pose.orientation.z;

        // Computation of the squared Mahalanobis distance
        double distance = pose_vector.transpose() * covariance_matrix * pose_vector;
        return distance;
    }

    geometry_msgs::PoseWithCovariance PairwiseConsistency::composeOnTrajectory(const size_t& id1, const size_t& id2) {
        // Extraction of the poses on the trajectory
        graph_utils::TrajectoryPose pose1 = (*trajectory_.find(id1)).second;
        graph_utils::TrajectoryPose pose2 = (*trajectory_.find(id2)).second;
        // Computation of the transformation
        geometry_msgs::PoseWithCovariance result;
        graph_utils::poseInverseCompose(pose2.pose, pose1.pose, result);
        return result;
    }
}