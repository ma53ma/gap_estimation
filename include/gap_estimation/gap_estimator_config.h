#ifndef GAP_ESTIMATOR_CONFIG_H
#define GAP_ESTIMATOR_CONFIG_H

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <boost/thread/mutex.hpp>

namespace gap_estimation {

    class GapEstimatorConfig {
        public:
            std::string map_frame_id;
            std::string odom_frame_id;
            std::string robot_frame_id;
            std::string sensor_frame_id;

            struct GapVisualization {
                int min_resoln;
                bool close_gap_vis;
                bool follow_the_gap_vis;
                bool fig_gen;
                double viz_jitter;
                bool debug_viz;
            } gap_viz;

            struct GapManipulation {
                double gap_diff;
                double epsilon2;
                double epsilon1;
                double sigma;
                double reduction_threshold;
                double reduction_target;
                int max_idx_diff;
                bool radial_extend;
                bool axial_convert;
                double rot_ratio;
                double cbf_param;
                double cbf_r_min;
                double K_des;
                double K_acc;
            } gap_manip;

            struct GapAssociation {
                double assoc_thresh;
            } gap_assoc;

            struct Waypoint {
                int global_plan_lookup_increment;
                double global_plan_change_tolerance;
            } waypoint;

            struct PlanningMode {
                bool feasi_inflated;  
                bool projection_inflated;
                bool planning_inflated;
                bool holonomic;
                bool full_fov;
                bool projection_operator;
                bool niGen_s;
                bool far_feasible;
                int num_feasi_check;
                int halt_size;
            } planning;

            struct Robot {
                float r_inscr;
                int num_obsts;
            } rbt;


        GapEstimatorConfig() {
            map_frame_id = "known_map";
            odom_frame_id = "map_static";
            robot_frame_id = "robot2";
            sensor_frame_id = "robot2_laser_0";

            gap_viz.min_resoln = 1;
            gap_viz.close_gap_vis = false;
            gap_viz.follow_the_gap_vis = false;
            gap_viz.fig_gen = false;
            gap_viz.viz_jitter = 0.1;
            gap_viz.debug_viz = true;

            gap_assoc.assoc_thresh = 0.5;

            planning.feasi_inflated = false;
            planning.projection_inflated = false;
            planning.planning_inflated = false;
            planning.holonomic = false;
            planning.full_fov = false;
            planning.projection_operator = true;
            planning.niGen_s = false;
            planning.num_feasi_check = 10;
            planning.far_feasible = false;
            planning.halt_size = 5;

            rbt.r_inscr = 0.2;
            rbt.num_obsts = 2;
        }

        void loadRosParamFromNodeHandle(ros::NodeHandle nh);

        boost::mutex & configMutex() {return config_mutex;}

        private: 
            boost::mutex config_mutex; 
    };
}

#endif