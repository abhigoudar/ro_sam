#pragma once

#include "fstream"
#include "iostream"
#include <unistd.h>
#include <sys/stat.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

namespace sam
{
    using namespace gtsam;
    static bool writeToFile(std::string file, const gtsam::Values& values)
    {
        // std::ofstream outfile;
        // outfile.open(file);

        // for(uint i = 1; i <(int)values.size(); i++)
        // {
        //     uint64_t key = symbol('x', i);
        //     Pose2 p = values.at<Pose2>(key);
        //     outfile << "x" + std::to_string(i) + ",";
        //     outfile << std::to_string(p.x()) + ",";
        //     outfile << std::to_string(p.y()) + ",";
        //     outfile << std::to_string(p.theta()) + "\n";
        // }

        // outfile.close();
    }

    static bool visualizeValues(std::string world_frame, const gtsam::Values& values)
    {
        std::vector<tf::StampedTransform> transforms;
        static tf::TransformBroadcaster broadcaster;
        
        for(uint i = 1; i <(int)values.size(); i++)
        {
            uint64_t key = symbol('x', i);
            Pose2 p = values.at<Pose2>(key);

            tf::StampedTransform transform;
            transform.stamp_ = ros::Time::now();
            transform.frame_id_ = world_frame;
            transform.child_frame_id_ = "x" + std::to_string(i);
            transform.setOrigin(tf::Vector3(p.x(), p.y(), 0.));
            transform.setRotation(tf::createQuaternionFromYaw(p.theta()));
            transforms.push_back(transform);
        }

        ROS_INFO("Broadcasting transfroms.");
        broadcaster.sendTransform(transforms);
        ros::Duration(0.5).sleep();
        broadcaster.sendTransform(transforms);
    }
}