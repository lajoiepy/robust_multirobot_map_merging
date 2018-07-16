// Copyright (C) 2018 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#ifndef ROBOT_LOCAL_MAP_H
#define ROBOT_LOCAL_MAP_H

#include "robot_local_map/robot_measurements.h"

namespace robot_local_map {
    /**
     * Class that contains the local map of a single robot.
     */ 
    class RobotLocalMap : public RobotMeasurements {
      public:
        /**
         * Constructor
         */
        RobotLocalMap(const std::string & file_name);

        /**
         * Accessors
         */
        const graph_utils::Trajectory& getTrajectory() const;

      private:
        graph_utils::Trajectory trajectory_;
    };          
}

#endif