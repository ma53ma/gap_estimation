
#include <gap_estimation/gap_estimation.h>
#include <map>
#include <vector>
#include <cmath>

#include <random>

#include <geometry_msgs/TransformStamped.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/LaserScan.h>

namespace gap_estimation
{ 
    Planner::Planner()
    {
        ROS_INFO_STREAM("starting initialize");

        // Visualization Setup
        local_traj_pub = nh.advertise<geometry_msgs::PoseArray>("relevant_traj", 1);
        trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("pg_traj", 1);
        dyn_egocircle_pub = nh.advertise<sensor_msgs::LaserScan>("dyn_egocircle", 1);

        robot0_cmd_vel_pub = nh.advertise<geometry_msgs::PoseArray>("/robot0/cmd_vel", 1);
        robot1_cmd_vel_pub = nh.advertise<geometry_msgs::PoseArray>("/robot1/cmd_vel", 1);

        rbt_accel_sub = nh.subscribe(cfg.robot_frame_id + "/acc", 1, &Planner::robotAccCB, this);

        // TF Lookup setup
        tfListener = new tf2_ros::TransformListener(tfBuffer);
        _initialized = true;

        finder = new GapUtils(cfg);
        gapvisualizer = new GapVisualizer(nh, cfg);
        gapassociator = new GapAssociator(nh, cfg);

        map2rbt.transform.rotation.w = 1;
        rbt2map.transform.rotation.w = 1;
        odom2rbt.transform.rotation.w = 1;
        rbt2odom.transform.rotation.w = 1;
        rbt_in_rbt.pose.orientation.w = 1;
        rbt_in_rbt.header.frame_id = cfg.robot_frame_id;

        current_rbt_vel = geometry_msgs::Twist();
        rbt_vel_min1 = geometry_msgs::Twist();

        rbt_accel = geometry_msgs::Twist();
        rbt_accel_min1 = geometry_msgs::Twist();
        
        init_val = 0;
        model_idx = &init_val;
        prev_traj_switch_time = ros::WallTime::now().toSec();
        init_time = ros::WallTime::now().toSec(); 

        curr_right_model = NULL;
        curr_left_model = NULL;

        sharedPtr_pose = geometry_msgs::Pose();
        sharedPtr_previous_pose = sharedPtr_previous_pose;

        prev_pose_time = ros::WallTime::now().toSec(); 
        prev_scan_time = ros::WallTime::now().toSec(); 

        prev_timestamp = ros::Time::now();
        curr_timestamp = ros::Time::now();

        final_goal_rbt = geometry_msgs::PoseStamped();
        num_obsts = cfg.rbt.num_obsts;

        agent_odoms = std::vector<geometry_msgs::Pose>(num_obsts);
        agent_odom_vects.resize(num_obsts, std::vector<double>(2));
        agent_vels = std::vector<geometry_msgs::Vector3Stamped>(num_obsts);
        agent_vel_vects.resize(num_obsts, std::vector<double>(2));
        for (int i = 0; i < num_obsts; i++) {
            ros::Subscriber temp_odom_sub = nh.subscribe("/robot" + to_string(i) + "/odom", 1, &Planner::agentOdomCB, this);
            agent_odom_subscribers.push_back(temp_odom_sub);
        }
    }

    Planner::~Planner() {} 


    // Acceleration message comes in in robot frame 
    void Planner::robotAccCB(boost::shared_ptr<geometry_msgs::Twist const> msg)
    {
        rbt_accel = *msg;
    }

    // running at point_scan rate which is around 8-9 Hz
    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        boost::mutex::scoped_lock gapset(gapset_mutex); // this is where time lag happens (~0.1 to 0.2 seconds)

        Matrix<double, 1, 3> v_ego(current_rbt_vel.linear.x, current_rbt_vel.linear.y, current_rbt_vel.angular.z);
        Matrix<double, 1, 3> a_ego(rbt_accel.linear.x, rbt_accel.linear.y, rbt_accel.angular.z);

        previous_raw_gaps = associated_raw_gaps;
        raw_gaps = finder->hybridScanGap(msg, final_goal_rbt);

        raw_distMatrix = gapassociator->obtainDistMatrix(raw_gaps, previous_raw_gaps, "raw");
        raw_association = gapassociator->associateGaps(raw_distMatrix);         // ASSOCIATE GAPS PASSES BY REFERENCE
        gapassociator->assignModels(raw_association, raw_distMatrix, raw_gaps, previous_raw_gaps, v_ego, model_idx);
        associated_raw_gaps = update_models(raw_gaps, v_ego, a_ego, false);
        
        previous_gaps = associated_observed_gaps;
        observed_gaps = finder->mergeGapsOneGo(msg, raw_gaps);
        
        simp_distMatrix = gapassociator->obtainDistMatrix(observed_gaps, previous_gaps, "simplified"); // finishes
        simp_association = gapassociator->associateGaps(simp_distMatrix); // must finish this and therefore change the association
        gapassociator->assignModels(simp_association, simp_distMatrix, observed_gaps, previous_gaps, v_ego, model_idx);
        associated_observed_gaps = update_models(observed_gaps, v_ego, a_ego, true);

        gapvisualizer->drawGaps(associated_raw_gaps, std::string("raw"));
        gapvisualizer->drawGaps(associated_observed_gaps, std::string("simp"));
        gapvisualizer->drawGapsModels(associated_observed_gaps);

        rbt_vel_min1 = current_rbt_vel;
        rbt_accel_min1 = rbt_accel;

    }

    void Planner::update_model(int i, std::vector<Gap>& _observed_gaps, Matrix<double, 1, 3> _v_ego, Matrix<double, 1, 3> _a_ego, bool print) {
        // boost::mutex::scoped_lock gapset(gapset_mutex);
        Gap g = _observed_gaps[int(std::floor(i / 2.0))];

        // UPDATING MODELS
        //std::cout << "obtaining gap g" << std::endl;
        geometry_msgs::Vector3Stamped gap_pt_vector_rbt_frame;
        geometry_msgs::Vector3Stamped range_vector_rbt_frame;
        gap_pt_vector_rbt_frame.header.frame_id = cfg.robot_frame_id;
        //std::cout << "obtaining gap pt vector" << std::endl;
        // THIS VECTOR IS IN THE ROBOT FRAME
        if (i % 2 == 0) {
            gap_pt_vector_rbt_frame.vector.x = g.RDist() * cos(-((float) g.half_scan - g.RIdx()) / g.half_scan * M_PI);
            gap_pt_vector_rbt_frame.vector.y = g.RDist() * sin(-((float) g.half_scan - g.RIdx()) / g.half_scan * M_PI);
        } else {
            gap_pt_vector_rbt_frame.vector.x = g.LDist() * cos(-((float) g.half_scan - g.LIdx()) / g.half_scan * M_PI);
            gap_pt_vector_rbt_frame.vector.y = g.LDist() * sin(-((float) g.half_scan - g.LIdx()) / g.half_scan * M_PI);
        }

        range_vector_rbt_frame.vector.x = gap_pt_vector_rbt_frame.vector.x - rbt_in_cam.pose.position.x;
        range_vector_rbt_frame.vector.y = gap_pt_vector_rbt_frame.vector.y - rbt_in_cam.pose.position.y;
        //std::cout << "gap_pt_vector_rbt_frame: " << gap_pt_vector_rbt_frame.vector.x << ", " << gap_pt_vector_rbt_frame.vector.y << std::endl;
        
        // pretty sure rbt in cam is always 0,0
        //std::cout << "rbt in cam pose: " << rbt_in_cam.pose.position.x << ", " << rbt_in_cam.pose.position.x << std::endl;
        //std::cout << "range vector rbt frame: " << range_vector_rbt_frame.vector.x << ", " << range_vector_rbt_frame.vector.x << std::endl;

        double beta_tilde = std::atan2(range_vector_rbt_frame.vector.y, range_vector_rbt_frame.vector.x);
        double range_tilde = std::sqrt(std::pow(range_vector_rbt_frame.vector.x, 2) + pow(range_vector_rbt_frame.vector.y, 2));
        
        Matrix<double, 2, 1> laserscan_measurement(range_tilde, beta_tilde);

        if (i % 2 == 0) {
            //std::cout << "entering left model update" << std::endl;
            g.right_model->kf_update_loop(laserscan_measurement, _a_ego, _v_ego, print, agent_odom_vects, agent_vels);
        } else {
            //std::cout << "entering right model update" << std::endl;
            g.left_model->kf_update_loop(laserscan_measurement, _a_ego, _v_ego, print, agent_odom_vects, agent_vels);
        }
    }

    // TO CHECK: DOES ASSOCIATIONS KEEP OBSERVED GAP POINTS IN ORDER (0,1,2,3...)
    std::vector<Gap> Planner::update_models(std::vector<Gap> _observed_gaps, Matrix<double, 1, 3> _v_ego, Matrix<double, 1, 3> _a_ego, bool print) {
        std::vector<Gap> associated_observed_gaps = _observed_gaps;
        
        // double start_time = ros::WallTime::now().toSec();
        for (int i = 0; i < 2*associated_observed_gaps.size(); i++) {
            //std::cout << "update gap model: " << i << std::endl;
            update_model(i, associated_observed_gaps, _v_ego, _a_ego, print);
            //std::cout << "" << std::endl;
        }

        //ROS_INFO_STREAM("update_models time elapsed: " << ros::WallTime::now().toSec() - start_time);
        return associated_observed_gaps;
    }

    void Planner::poseCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
        updateTF();
        //double curr_time = ros::WallTime::now().toSec();
        //std::cout << "pose rate: " << 1.0 / (curr_time - prev_pose_time) << std::endl;
        //prev_pose_time = curr_time;
        
        // Transform the msg to odom frame
        if(msg->header.frame_id != cfg.odom_frame_id)
        {
            //std::cout << "odom msg is not in odom frame" << std::endl;
            geometry_msgs::TransformStamped robot_pose_odom_trans = tfBuffer.lookupTransform(cfg.odom_frame_id, msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = msg->pose.pose;

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;

            tf2::doTransform(in_pose, out_pose, robot_pose_odom_trans);
            sharedPtr_pose = out_pose.pose;
        }
        else
        {
            sharedPtr_pose = msg->pose.pose;
        }

        current_rbt_vel = msg->twist.twist;
        
    }

    void Planner::agentOdomCB(const nav_msgs::Odometry::ConstPtr& msg) {

        std::string robot_namespace = msg->child_frame_id;
        ROS_INFO_STREAM("agentOdomCB robot_namespace: " << robot_namespace);
        robot_namespace.erase(0,5); // removing "robot"
        char *robot_name_char = strdup(robot_namespace.c_str());
        int robot_id = std::atoi(robot_name_char);
        ROS_INFO_STREAM("agentOdomCB robot_id: " << robot_id);
        // I need BOTH odom and vel in robot2 frame
        //std::cout << "odom msg is not in odom frame" << std::endl;
        try {
            // transforming Odometry message from map_static to robotN
            geometry_msgs::TransformStamped agent_to_robot_odom_trans = tfBuffer.lookupTransform(cfg.robot_frame_id, msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = msg->pose.pose;
            // ROS_INFO_STREAM("updating inpose " << robot_namespace << " to: (" << in_pose.pose.position.x << ", " << in_pose.pose.position.y << ")");

            //std::cout << "rbt vel: " << msg->twist.twist.linear.x << ", " << msg->twist.twist.linear.y << std::endl;

            tf2::doTransform(in_pose, out_pose, agent_to_robot_odom_trans);
            // ROS_INFO_STREAM("updating outpose " << robot_namespace << " from: (" << agent_odoms[robot_id].position.x << ", " << agent_odoms[robot_id].position.y << ") to: (" << out_pose.pose.position.x << ", " << out_pose.pose.position.y << ")");
            // ROS_INFO_STREAM("updating " << robot_namespace << " pose from: (" << agent_odom_vects[robot_id][0] << ", " << agent_odom_vects[robot_id][1] << ") to: (" << out_pose.pose.position.x << ", " << out_pose.pose.position.y << ")");

            std::vector<double> odom_vect{out_pose.pose.position.x, out_pose.pose.position.y};
            agent_odom_vects[robot_id] = odom_vect;
            agent_odoms[robot_id] = out_pose.pose;
        } catch (tf2::TransformException &ex) {
            ROS_INFO_STREAM("Odometry transform failed for " << robot_namespace);
        }
        
        try {
            std::string source_frame = msg->child_frame_id; 
            // std::cout << "in agentOdomCB" << std::endl;
            // std::cout << "transforming from " << source_frame << " to " << cfg.robot_frame_id << std::endl;
            geometry_msgs::TransformStamped agent_to_robot_trans = tfBuffer.lookupTransform(cfg.robot_frame_id, source_frame, ros::Time(0));
            geometry_msgs::Vector3Stamped in_vel, out_vel;
            in_vel.header = msg->header;
            in_vel.header.frame_id = source_frame;
            in_vel.vector = msg->twist.twist.linear;
            // std::cout << "incoming vector: " << in_vel.vector.x << ", " << in_vel.vector.y << std::endl;
            tf2::doTransform(in_vel, out_vel, agent_to_robot_trans);
            // std::cout << "outcoming vector: " << out_vel.vector.x << ", " << out_vel.vector.y << std::endl;

            // ROS_INFO_STREAM("updating outvel " << robot_namespace << " from: (" << agent_vels[robot_id].vector.x << ", " << agent_vels[robot_id].vector.y << ") to: (" << out_vel.vector.x << ", " << out_vel.vector.y << ")");
            // ROS_INFO_STREAM("updating outvel vect " << robot_namespace << " from: (" << agent_vel_vects[robot_id][0] << ", " << agent_vel_vects[robot_id][1] << ") to: (" << out_vel.vector.x << ", " << out_vel.vector.y << ")");

            std::vector<double> vel_vect{out_vel.vector.x, out_vel.vector.y};

            agent_vels[robot_id] = out_vel;
            agent_vel_vects[robot_id] = vel_vect;
        } catch (tf2::TransformException &ex) {
            ROS_INFO_STREAM("Velocity transform failed for " << robot_namespace);
        }            

        geometry_msgs::Twist cmd_twist;
        if (robot_id == 0) {
            ROS_INFO_STREAM("publishing cmd vel to robot0");
            cmd_twist.linear.y = -0.3;
            robot0_cmd_vel_pub.publish(cmd_twist);
        } else if (robot_id == 1) {
            ROS_INFO_STREAM("publishing cmd vel to robot1");
            cmd_twist.linear.y = 0.3;
            robot1_cmd_vel_pub.publish(cmd_twist);
        }
    }

    void Planner::updateTF()
    {
        try {
            map2rbt  = tfBuffer.lookupTransform(cfg.robot_frame_id, cfg.map_frame_id, ros::Time(0));
            rbt2map  = tfBuffer.lookupTransform(cfg.map_frame_id, cfg.robot_frame_id, ros::Time(0));
            odom2rbt = tfBuffer.lookupTransform(cfg.robot_frame_id, cfg.odom_frame_id, ros::Time(0));
            rbt2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.robot_frame_id, ros::Time(0));
            cam2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.sensor_frame_id, ros::Time(0));
            map2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.map_frame_id, ros::Time(0));
            rbt2cam = tfBuffer.lookupTransform(cfg.sensor_frame_id, cfg.robot_frame_id, ros::Time(0));

            tf2::doTransform(rbt_in_rbt, rbt_in_cam, rbt2cam);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            return;
        }
    }
}

