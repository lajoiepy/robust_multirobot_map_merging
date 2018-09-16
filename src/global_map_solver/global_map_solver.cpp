// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "global_map_solver/global_map_solver.h"
#include "findClique.h"
#include <math.h>


namespace global_map_solver {

const std::string GlobalMapSolver::CONSISTENCY_MATRIX_FILE_NAME = std::string("results/consistency_matrix.clq.mtx");
const std::string GlobalMapSolver::CONSISTENCY_LOOP_CLOSURES_FILE_NAME = std::string("results/consistent_loop_closures.txt");

GlobalMapSolver::GlobalMapSolver(const robot_local_map::RobotLocalMap& robot1_local_map,
                const robot_local_map::RobotLocalMap& robot2_local_map,
                const robot_local_map::RobotMeasurements& interrobot_measurements): 
                pairwise_consistency_(robot1_local_map.getTransforms(), robot2_local_map.getTransforms(), 
                            interrobot_measurements.getTransforms(), interrobot_measurements.getLoopClosures(),
                            robot1_local_map.getTrajectory(), robot2_local_map.getTrajectory(),
                            robot1_local_map.getNbDegreeFreedom()){}

SESync::measurements_t GlobalMapSolver::fillMeasurements(const std::vector<int>& max_clique_data){

    // Preallocate output vector
    SESync::measurements_t measurements;

    for (auto const& t : pairwise_consistency_.getTransformsRobot1().transforms)
    {
        measurements.push_back(graph_utils::convertTransformToRelativePoseMeasurement(t.second));
    }

    for (auto const& t : pairwise_consistency_.getTransformsRobot2().transforms)
    {
        measurements.push_back(graph_utils::convertTransformToRelativePoseMeasurement(t.second));
    }

    for (auto const& t : pairwise_consistency_.getTransformsInterRobot().transforms)
    {
        measurements.push_back(graph_utils::convertTransformToRelativePoseMeasurement(t.second));
    }

    return measurements;
}

int GlobalMapSolver::solveGlobalMap() {
    // Compute consistency matrix
    Eigen::MatrixXi consistency_matrix = pairwise_consistency_.computeConsistentMeasurementsMatrix();
    graph_utils::printConsistencyGraph(consistency_matrix, CONSISTENCY_MATRIX_FILE_NAME);
    
    // Compute maximum clique
    FMC::CGraphIO gio;
    gio.readGraph(CONSISTENCY_MATRIX_FILE_NAME);
    int max_clique_size = 0;
    std::vector<int> max_clique_data;
    max_clique_size = FMC::maxClique(gio, max_clique_size, max_clique_data);

    // Print results
    graph_utils::printConsistentLoopClosures(pairwise_consistency_.getLoopClosures(), max_clique_data, CONSISTENCY_LOOP_CLOSURES_FILE_NAME);

    // Clean up
    max_clique_data.clear();

    // Fill measurements
    SESync::measurements_t measurements = fillMeasurements(max_clique_data);
    
    // SE-Sync options
    SESync::SESyncOpts opts;
    opts.verbose = true;
    opts.num_threads = 4;

    /// RUN SE-SYNC! (optimization)
    SESync::SESyncResult results = SESync::SESync(measurements, opts);    

    return max_clique_size;
}

}