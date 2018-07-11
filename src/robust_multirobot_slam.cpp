#include "robust_multirobot_slam.h"
#include "pose_cov_ops/pose_cov_ops.h"

namespace robust_multirobot_slam {

    Eigen::MatrixXd computeConsistentMeasurementsMatrix(const std::map<std::pair<size_t,size_t>, graph_utils::Transform>& transforms,
                                                        const std::list<std::pair<size_t,size_t>>& loop_closure_list) {
        // Preallocate consistency matrix
        Eigen::MatrixXd consistency_matrix(loop_closure_list.size(), loop_closure_list.size());

        // Iterate on loop closures
        size_t u = 0;
        for (std::list<std::pair<size_t,size_t>>::const_iterator it_row = loop_closure_list.begin(); it_row != loop_closure_list.end(); ++it_row) {
            size_t v = 0;
            for (std::list<std::pair<size_t,size_t>>::const_iterator it_col = loop_closure_list.begin(); it_col != loop_closure_list.end(); ++it_col) {
                if (u != v) {
                    size_t i,j,k,l;
                    i = (*it_row).first;
                    k = (*it_row).second;
                    j = (*it_col).first;
                    l = (*it_col).second;
                    geometry_msgs::PoseWithCovariance abZik = (*transforms.find(*it_row)).second.pose;
                    geometry_msgs::PoseWithCovariance abZjl = (*transforms.find(*it_col)).second.pose; 
                    geometry_msgs::PoseWithCovariance aXij = (*transforms.find(std::make_pair(i,j))).second.pose;  
                    geometry_msgs::PoseWithCovariance bXlk = (*transforms.find(std::make_pair(k,l))).second.pose; 
                    geometry_msgs::PoseWithCovariance consistency_pose = computeConsistencyPose(aXij, bXlk, abZik, abZjl);
                    double distance = computeSquaredMahalanobisDistance(consistency_pose);
                    consistency_matrix(u,v) = distance;
                    // TODO: thresholding
                }
                v++;
            }
            u++;
        }
        return consistency_matrix;
    }
    
    geometry_msgs::PoseWithCovariance computeConsistencyPose(const geometry_msgs::PoseWithCovariance& aXij, 
                                                            const geometry_msgs::PoseWithCovariance& bXlk, 
                                                            const geometry_msgs::PoseWithCovariance& abZik, 
                                                            const geometry_msgs::PoseWithCovariance& abZjl) {
        // Consistency loop : aXij + abZjl + bXlk - abZik
        geometry_msgs::PoseWithCovariance out1, out2, result;
        pose_cov_ops::compose(aXij, abZjl, out1);
        pose_cov_ops::compose(out1, bXlk, out2);
        pose_cov_ops::inverseCompose(out2, abZik, result);

        return result;
    }

    double computeSquaredMahalanobisDistance(const geometry_msgs::PoseWithCovariance& pose) {
        // Extraction of the covariance matrix
        Eigen::Matrix<double, 6, 6> covariance_matrix;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                covariance_matrix(i,j) = pose.covariance[i*6+j];
            }
        }

        // Extraction of the pose vector
        Eigen::VectorXd pose_vector(6);
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

    geometry_msgs::PoseWithCovariance composeOnTrajectory(const size_t& first_pose_id, const size_t& second_pose_id, 
                                const std::map<std::pair<size_t,size_t>, graph_utils::Transform>& transforms) {
        // Make sure to compose in ascending order
        size_t start_pose_id, end_pose_id;
        if (first_pose_id < second_pose_id) {
            start_pose_id = first_pose_id;
            end_pose_id = second_pose_id;
        } else {
            start_pose_id = second_pose_id;
            end_pose_id = first_pose_id;
        }

        // Initialization
        std::pair<size_t, size_t> start_pair = std::make_pair(start_pose_id, start_pose_id+1);
        size_t current_pose_id = start_pose_id + 1;
        geometry_msgs::PoseWithCovariance temp_pose, total_pose;
        temp_pose = (*transforms.find(start_pair)).second.pose;

        // Compositions in chain
        while (current_pose_id < end_pose_id) {
            std::pair<size_t, size_t> temp_pair = std::make_pair(current_pose_id, current_pose_id + 1);
            pose_cov_ops::compose(temp_pose, (*transforms.find(temp_pair)).second.pose, total_pose);
            temp_pose = total_pose;
            current_pose_id++;
        }

        return total_pose;
    }
}