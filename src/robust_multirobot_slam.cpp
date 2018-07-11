#include "robust_multirobot_slam.h"

namespace robust_multirobot_slam {

    Eigen::MatrixXd computeConsistentMeasurementsMatrix(const std::vector<graph_utils::Transform>& transforms) {

    }
    
    geometry_msgs::PoseWithCovariance computeConsistencyPose(const geometry_msgs::PoseWithCovariance& aXij, 
                                                            const geometry_msgs::PoseWithCovariance& bXlk, 
                                                            const geometry_msgs::PoseWithCovariance& abZik, 
                                                            const geometry_msgs::PoseWithCovariance& abZjl) {

    }

    double computeSquaredMahalanobisDistance(const geometry_msgs::PoseWithCovariance& pose) {
        
    }

}