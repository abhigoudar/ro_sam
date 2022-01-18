#pragma once

#include "math_utilities/math_utils.h"
#include "ro_sam_2d/utils.h"

#include <ros/ros.h>
#include <rosbag_utils/parse_sensor_data.h>

#include <uwb_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include <data_types/measurement_base.h>
#include <data_types/uwb_range.h>
#include <data_types/twist.h>

#include "ro_sam_2d/uwb_unary_factor.h"
#include "ro_sam_2d/uwb_lever_arm_unary_factor.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>

#include <queue>

namespace sam
{
    class RoSam
    {
        public:
        /**
         * @brief default constructor
         * @param private nodehandle
         * @param global nodehandle
         */
        RoSam(const ros::NodeHandle&, const ros::NodeHandle&);
        /**
         * @brief destructor
         */
        ~RoSam();
        
        private:
        // private nodehandle
        ros::NodeHandle p_nh;
        // global nodehandle
        ros::NodeHandle nh;
        // subscriber for range data
        std::map<std::string, ros::Subscriber> sub_map;
        // publishers
        std::map<std::string, ros::Publisher> pub_map;
        // callback for range data
        void uwbCB(const uwb_msgs::Range::ConstPtr&);
        // callback for odometry data
        void odomCB(const nav_msgs::Odometry::ConstPtr&);
        // load parameters from ROS server
        void loadParams();
        // initialize local variables
        void initLocalVariables();
        // load offline batch data
        void loadOfflineData(gtsam::Vector3&, measurement::MeasurementVector&);
        // sample offline data
        void sampleOfflineData(const measurement::MeasurementVector&, int,
            measurement::MeasurementQueue&);
        // generate prior for batch
        void solveBatch(gtsam::Vector3&, measurement::MeasurementQueue&);
        // optimization routine
        void run();
        //
        void addAnchorPos(measurement::MeasurementVector& sensor_data);
        // node name
        std::string node_name;
        // operating mode: offline or online
        std::string mode;
    };
}