#include "ro_sam_2d/ro_sam.h"

namespace sam
{
    using namespace gtsam;
    RoSam::RoSam(const ros::NodeHandle& _p_nh, const ros::NodeHandle& _nh)
    {
        p_nh = _p_nh;
        nh = _nh;
        node_name = ros::this_node::getName().c_str(); 
        // load parameters from ROS 
        loadParams();
        //initialize local variables
        initLocalVariables();
        // run sam
        run();
    }

    RoSam::~RoSam()
    {

    }

    void RoSam::loadParams()
    {
        p_nh.param<std::string>("mode", mode, "offline");
        ROS_INFO("[%s]: Mode:[%s]", node_name.c_str(), mode.c_str());
    }

    void RoSam::initLocalVariables()
    {

    }

    void RoSam::run()
    {
        // offline mode
        if(mode.compare("offline") == 0)
        {
            Vector3 init_pose;
            measurement::MeasurementQueue meas_queue;
            measurement::MeasurementVector sensor_data;
            sensor_data.clear();
            meas_queue = measurement::MeasurementQueue();

            ROS_INFO("[%s]: RUNNING OFFLINE MODE.", node_name.c_str());
            loadOfflineData(init_pose, sensor_data);
            // this needs to be done in a better way
            addAnchorPos(sensor_data);
            // get sample factor
            int sample_factor;
            p_nh.param<int>("sample_factor", sample_factor, 0);
            ROS_INFO("[%s]: Sample factor:[%d]", node_name.c_str(), sample_factor);
            // sample raw sensor data
            sampleOfflineData(sensor_data, sample_factor, meas_queue);
            // generate prior
            solveBatch(init_pose, meas_queue);
            return;
        }

        // // online mode
        // while(ros::ok())
        // {

        // }
    }

    void RoSam::addAnchorPos(measurement::MeasurementVector& sensor_data)
    {
        std::map<std::string, Vector3> anchor_dict;
        anchor_dict["anchor_1"] = Vector3(10, 10, 10);
        anchor_dict["anchor_2"] = Vector3(-10, 10, 10);
        anchor_dict["anchor_3"] = Vector3(-10, -10, 10);
        anchor_dict["anchor_4"] = Vector3(10, -10, 10);  
        for(int i = 0; i < sensor_data.size(); i++)
        {
            if(sensor_data.at(i)->type == measurement::UWB_RANGE)
            {
                measurement::UWBRangePtr range = 
                    std::dynamic_pointer_cast<measurement::UWBRange>(sensor_data.at(i));
                range->anchorPos.push_back(anchor_dict[range->anchors[0]]);
            }
        }
    }

    void RoSam::loadOfflineData(Vector3& init_pose,
        measurement::MeasurementVector& sensor_data)
    {
        std::string bag_file;
        std::vector<std::string> topics;
        p_nh.param<std::string>("rosbag", bag_file, "");
        ROS_INFO("[%s]: ROSBAG from param server:[%s]", node_name.c_str(),
            bag_file.c_str());
        
        p_nh.param<std::vector<std::string> >("topics", topics, std::vector<std::string>{});
        ROS_INFO("[%s]: Parsing topics:", node_name.c_str());
        std::for_each(topics.begin(), topics.end(), [](const std::string& topic)
            {std::cout << topic << ",";});
        std::cout << std::endl;
        
        ros::Duration(0.5).sleep();

        if(rosbag_utils::parseSensorData(bag_file, topics, sensor_data))
            ROS_INFO("[%s]: Successfully prased sensor data.", node_name.c_str());

        // load initial pose
        p_nh.param<double>("initial_pose/position/x", init_pose[0], 0);
        p_nh.param<double>("initial_pose/position/y", init_pose[1], 0);
        p_nh.param<double>("initial_pose/orientation/yaw", init_pose[2], 0);
        ROS_INFO("[%s]: Initial pose:x:[%.3f] y:[%.3f] yaw:[%.3f]", node_name.c_str(),
            init_pose[0], init_pose[1], init_pose[2]);
    }

    void RoSam::sampleOfflineData(const measurement::MeasurementVector& sensor_data, int sample_factor,
        measurement::MeasurementQueue& meas_queue)
    {
        int count = 0;
        for(int i = 0; i < (int)sensor_data.size(); i++)
        {
            if(sensor_data.at(i)->type == measurement::UWB_RANGE)
            {
                if(count == sample_factor)
                {
                    // measurement::UWBRangePtr range = 
                    //     std::dynamic_pointer_cast<measurement::UWBRange>(sensor_data.at(i));
                    meas_queue.push(sensor_data.at(i));
                    count = 0;
                }
                else
                    count++;
            }
            else
                meas_queue.push(sensor_data.at(i));
        }

        ROS_INFO("[%s]: Raw sensor data length:[%d] Sampled data:[%d]", node_name.c_str(),
            (int)sensor_data.size(), (int)meas_queue.size());
    }

    void RoSam::solveBatch(Vector3& init_pose,
        measurement::MeasurementQueue& meas_queue)
    {
        using namespace gtsam;

        NonlinearFactorGraph graph;
        
        // to keep track of nodes in graph
        uint pose_index = 1;
        // add prior on first pose
        Pose2 prior_pose(init_pose[0], init_pose[1], init_pose[2]);
        Point2 lev_arm_init(0.15,-0.15);

        noiseModel::Diagonal::shared_ptr prior_noise_pose = 
            noiseModel::Diagonal::Sigmas(Vector3(0.1,0.1,0.1));
        noiseModel::Diagonal::shared_ptr prior_noise_leva = 
            noiseModel::Diagonal::Sigmas(Vector2(0.1,0.1));
        graph.add(PriorFactor<Pose2>(symbol('x', pose_index),
            prior_pose, prior_noise_pose));
        graph.add(PriorFactor<Point2>(symbol('l', pose_index),
            lev_arm_init, prior_noise_leva));
        
        pose_index++;

        // keep track of previous pose
        Pose2 prev_pose;
        prev_pose = prior_pose;
        // object to calculate between factor
        Vector3 acc_odom(0,0,0);
        Matrix3 acc_odom_cov;
        acc_odom_cov.fill(0);
        // initial guess for lever arm
        Values initial;
        initial.insert(symbol('x',pose_index-1), prior_pose);
        initial.insert(symbol('l',pose_index-1), lev_arm_init);

        // time stamp o[f previous odometry message
        double prev_stamp = -1;
        measurement::ImuPtr latestImu;
        while(!meas_queue.empty())
        {
            measurement::MeasurementBasePtr meas = meas_queue.top();
            
            switch(meas->type)
            {
                case measurement::UWB_RANGE:
                {
                    measurement::UWBRangePtr range = std::dynamic_pointer_cast<measurement::UWBRange>(meas);
                    // create between factor
                    Pose2 rel_pose(acc_odom[0], acc_odom[1], acc_odom[2]);
                    Pose2 prop_pose(prev_pose*rel_pose);
                    noiseModel::Diagonal::shared_ptr odometryNoise =
                        noiseModel::Diagonal::Sigmas(Vector3(0.01,
                            0.01, 0.01));//use actual covariance values from G
                    // current node
                    uint64_t curr_key_pose = symbol('x', pose_index);
                    uint64_t curr_key_leva = symbol('l', pose_index);
                    // previous node
                    uint64_t prev_key = symbol('x', pose_index-1);
                    // create unary factor at current note
                    noiseModel::Diagonal::shared_ptr unaryNoise =
                        noiseModel::Diagonal::Sigmas(Vector1(0.0001));
                    // add between factor
                    graph.add(BetweenFactor<Pose2>(prev_key, curr_key_pose, rel_pose,
                        odometryNoise));
                    // add unary factor
                    // double pred_range = (Vector2(range->anchorPos[0][0],range->anchorPos[0][1])
                    //     - Vector2(prop_pose.x(), prop_pose.y())).norm();
                    // printf("Adding anchor:[%s] x:[%f] y:[%f] mobile:x:[%f] y:[%f] range:[%f] pred_range:[%f] res:[%f]\n",
                    //     range->anchors[0].c_str(), range->anchorPos[0][0], range->anchorPos[0][1], prop_pose.x(),
                    //     prop_pose.y(), range->data[0], pred_range, range->data[0] - pred_range);
                    graph.add(boost::make_shared<UWBRangeLeverArmFactor2D>(curr_key_pose, 
                        curr_key_leva, unaryNoise, range->anchorPos[0], range->data[0]));
                    // initial value for current node
                    // corrupt the initial guess
                    Pose2 prop_pose_per(prop_pose.x(), prop_pose.y(), prop_pose.theta());
                    initial.insert(curr_key_pose, prop_pose_per);
                    initial.insert(curr_key_leva, lev_arm_init);
                    // update previous pose
                    prev_pose = prop_pose;
                    // reset odometry
                    acc_odom.fill(0);
                    acc_odom_cov.fill(0);
                    // increase node index
                    pose_index++;
                    break;
                }
                case measurement::ODOMETRY:
                {
                    measurement::OdometryPtr odom = std::dynamic_pointer_cast<measurement::Odometry>(meas);
                    if(prev_stamp < 0)
                        prev_stamp = odom->header.stamp;

                    double dt = odom->header.stamp - prev_stamp;
                    double v = odom->twist.linear[0];
                    double w;
                    if(latestImu)
                    {
                        w = latestImu->angular_velocity[2];
                    }
                    else
                        w = odom->twist.angular[2];
                    double cp = cos(acc_odom[2]);
                    double sp = sin(acc_odom[2]);

                    // Predict state forward
                    acc_odom[0] = acc_odom[0] + v * cp * dt;
                    acc_odom[1] = acc_odom[1] + v * sp * dt;
                    acc_odom[2] = acc_odom[2] + w * dt;
                    math_utils::clampRotation(acc_odom[2]);

                    Matrix32 G;
                    G(0,0) = dt * cp;
                    G(1,0) = dt * sp;
                    G(2,1) = dt;
                    acc_odom_cov.noalias() = G*(Matrix2::Identity()*0.01)*G.transpose();
                    // figure out how to add whole covariance matrix
                    prev_stamp = odom->header.stamp;
                    break;
                }
                case measurement::IMU:
                {
                    latestImu = std::dynamic_pointer_cast<measurement::Imu>(meas);
                    break;
                }
                default:
                {
                    break;
                }
            }
            meas_queue.pop();
        }
        std::cout << "Number of poses:" << pose_index-1 << std::endl;
        // graph.print("\nFactor Graph:\n");  // print
        // initial.print("\nInitial Estimate:\n");  // print
        std::string file;
        p_nh.param<std::string>("rosbag", file, "");
        std::string initial_values_file, result_values_file;
        initial_values_file = file.substr(0, file.find_first_of('.')) + "_initial.csv";
        result_values_file = file.substr(0, file.find_first_of('.')) + "_result.csv";
        ROS_INFO("[%s]:Writing initial values to:[%s]", node_name.c_str(),
            initial_values_file.c_str());
        writeToFile(initial_values_file, initial);
        Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
        ROS_INFO("Final result:");
        result.print();
        // ROS_INFO("[%s]:Writing final values to:[%s]", node_name.c_str(),
        //     result_values_file.c_str());
        //writeToFile(result_values_file, result);
        //visualizeValues("world", result);
    }
} // namespace sam


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ro_sam_node");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    sam::RoSam rosam(p_nh, nh);
    ros::spin();
    return 0;
}