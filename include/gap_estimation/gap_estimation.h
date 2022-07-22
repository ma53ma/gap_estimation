#ifndef GAP_ESTIMATION_H
#define GAP_ESTIMATION_H

#include <gap_estimation/gap.h>
#include <gap_estimation/gap_utils.h>
#include <gap_estimation/visualization.h>
#include <gap_estimation/gap_associator.h>
#include <gap_estimation/gap_estimator_config.h>


#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <math.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Header.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace gap_estimation
{
  class Planner
  {
    private:
      geometry_msgs::TransformStamped map2rbt;        // Transform
      geometry_msgs::TransformStamped rbt2map;
      geometry_msgs::TransformStamped odom2rbt;
      geometry_msgs::TransformStamped rbt2odom;
      geometry_msgs::TransformStamped map2odom;
      geometry_msgs::TransformStamped cam2odom;
      geometry_msgs::TransformStamped rbt2cam;

      geometry_msgs::PoseStamped goal_rbt_frame;
      geometry_msgs::PoseStamped curr_pose_odom;
      geometry_msgs::PoseStamped rbt_in_rbt;
      geometry_msgs::PoseStamped rbt_in_cam;
      geometry_msgs::Twist rbt_vel_in_rbt;

      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener *tfListener;
      tf2_ros::TransformBroadcaster goal_br;

      ros::NodeHandle nh;
      ros::Publisher local_traj_pub;
      ros::Publisher trajectory_pub;
      ros::Publisher gap_vis_pub;
      ros::Publisher selected_gap_vis_pub;
      ros::Publisher ni_traj_pub;
      ros::Publisher ni_traj_pub_other;
      ros::Publisher robot0_cmd_vel_pub;
      ros::Publisher robot1_cmd_vel_pub;

      ros::Publisher dyn_egocircle_pub;
      
      ros::Subscriber rbt_accel_sub;
      ros::Subscriber agent0_vel_sub;
      ros::Subscriber agent1_vel_sub;
      // ros::Subscriber rbt_vel_sub;

      std::vector<int> simp_association;
      std::vector<int> raw_association;

      std::vector< std::vector<double> > simp_distMatrix;
      std::vector< std::vector<double> > raw_distMatrix;

      bool print_associations;


      // Goals and stuff
      // double goal_orientation;
      geometry_msgs::Pose current_pose_;
      geometry_msgs::PoseStamped local_waypoint_odom; // local_waypoint, 
      geometry_msgs::PoseStamped final_goal_odom;
      geometry_msgs::PoseStamped final_goal_rbt;

      // Gaps:
      std::vector<gap_estimation::Gap> raw_gaps;
      std::vector<gap_estimation::Gap> observed_gaps;

      std::vector<gap_estimation::Gap> previous_gaps;
      std::vector<gap_estimation::Gap> previous_raw_gaps;

      std::vector<gap_estimation::Gap> associated_raw_gaps;
      std::vector<gap_estimation::Gap> associated_observed_gaps;

      gap_estimation::GapUtils *finder;
      gap_estimation::GapVisualizer *gapvisualizer;
      gap_estimation::GapAssociator *gapassociator;

      // Status
      bool hasGoal = false;
      bool _initialized = false;

      geometry_msgs::PoseArray pose_arr;
      geometry_msgs::PoseArray pose_arr_odom;

      // std::vector<turtlebot_trajectory_generator::ni_state> ctrl;
      int ctrl_idx = 0;

      geometry_msgs::Pose sharedPtr_pose;
      geometry_msgs::Pose sharedPtr_previous_pose;
      boost::shared_ptr<sensor_msgs::LaserScan const> static_scan_ptr;
      boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser;
      boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_inflatedlaser;

      ros::WallTime last_time;

      // Dynamic Reconfigure
      // boost::shared_ptr<dynamic_reconfigure::Server<gap_estimator_testing::dgConfig> > dynamic_recfg_server;
      // dynamic_reconfigure::Server<gap_estimator_testing::dgConfig>::CallbackType f;

      bool replan = true;
      
      GapEstimatorConfig cfg;

      boost::mutex gapset_mutex;

      geometry_msgs::PoseArray curr_executing_traj;
      std::vector<double> curr_executing_time_arr;
      int curr_exec_left_idx;
      int curr_exec_right_idx;

      geometry_msgs::Twist current_rbt_vel;
      geometry_msgs::Twist rbt_vel_min1;
      geometry_msgs::Twist rbt_accel;
      geometry_msgs::Twist rbt_accel_min1;

      std::string frame;
      double angle_increment;
      double angle_min;
      double half_scan;
      double min_safe_dist;

      int init_val;
      int * model_idx;
      double prev_traj_switch_time;
      double init_time;

      gap_estimation::cart_model * curr_right_model;
      gap_estimation::cart_model * curr_left_model;
      double curr_peak_velocity_x;
      double curr_peak_velocity_y;

      double prev_pose_time;
      double prev_scan_time;

      bool first_traj;
      double max_range = 4.9;

      ros::Time prev_timestamp;
      ros::Time curr_timestamp;

      double max_scan_time_elapsed;

      geometry_msgs::Pose robot0_odom;
      geometry_msgs::Vector3Stamped robot0_vel;

      geometry_msgs::Pose robot1_odom;
      geometry_msgs::Vector3Stamped robot1_vel;
      int num_obsts;

      std::vector< std::vector<double>> agent_odom_vects;
      std::vector<ros::Subscriber> agent_odom_subscribers;
      std::vector<geometry_msgs::Pose> agent_odoms;
      std::vector<geometry_msgs::Vector3Stamped> agent_vels;
      std::vector< std::vector<double>> agent_vel_vects;

  public:
      Planner();

      ~Planner();

      /**
          * call back function to laserscan, externally linked
          * @param msg laser scan msg
          * @return None, laser scan msg stored locally
          */
      void laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);
      void inflatedlaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);

      void staticLaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);

      void robotAccCB(boost::shared_ptr<geometry_msgs::Twist const> msg);
      // void robotVelCB(boost::shared_ptr<geometry_msgs::Twist const> msg);

      /**
          * call back function to pose, pose information obtained here only used when a new goal is used
          * @param msg pose msg
          * @return None
          */
      void poseCB(const nav_msgs::Odometry::ConstPtr& msg);

      /**
          * Interface function for receiving global plan
          * @param plan, vector of PoseStamped
          * @return boolean type on whether successfully registered goal
          */
      bool setGoal(const std::vector<geometry_msgs::PoseStamped> &plan);

      /**
          * update all tf transform at the beginning of every planning cycle
          * @param None, all tf received via TF
          * @return None, all registered via internal variables in TransformStamped
          */
      void updateTF();

      void update_model(int i, std::vector<gap_estimation::Gap>& _observed_gaps, Matrix<double, 1, 3> _v_ego, Matrix<double, 1, 3> _a_ego, bool print);
      std::vector<gap_estimation::Gap> update_models(std::vector<gap_estimation::Gap> _observed_gaps, Matrix<double, 1, 3> _v_ego, Matrix<double, 1, 3> _a_ego, bool print);
      std::vector<gap_estimation::Gap> get_curr_raw_gaps();
      std::vector<gap_estimation::Gap> get_curr_observed_gaps();

      void setCurrentRightModel(cart_model * _right_model);
      void setCurrentLeftModel(cart_model * _left_model);
      void setCurrentGapPeakVelocities(double _peak_velocity_x, double _peak_velocity_y);

      void printGapModels(std::vector<gap_estimation::Gap> gaps);
      void printGapAssociations(std::vector<gap_estimation::Gap> current_gaps, std::vector<gap_estimation::Gap> previous_gaps, std::vector<int> association);

      std::vector<int> get_raw_associations();

      std::vector<int> get_simplified_associations();


      void agentOdomCB(const nav_msgs::Odometry::ConstPtr& msg);

      int get_num_obsts();

  };
}

#endif // GAP_ESTIMATION_H
