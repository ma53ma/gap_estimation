#include <gap_estimation/gap_estimator_config.h>

namespace gap_estimation {

    void GapEstimatorConfig::loadRosParamFromNodeHandle(ros::NodeHandle nh)
    {
        // format: key, value, default value
        nh.param("map_frame_id", map_frame_id, map_frame_id);
        nh.param("odom_frame_id", odom_frame_id, odom_frame_id);
        nh.param("robot_frame_id", robot_frame_id, robot_frame_id);
        nh.param("sensor_frame_id", sensor_frame_id, sensor_frame_id);

        // Gap Visualization
        nh.param("min_resoln", gap_viz.min_resoln, gap_viz.min_resoln);
        nh.param("close_gap", gap_viz.close_gap_vis, gap_viz.close_gap_vis);
        nh.param("follow_the_gap", gap_viz.follow_the_gap_vis, gap_viz.follow_the_gap_vis);
        nh.param("fig_gen", gap_viz.fig_gen, gap_viz.fig_gen);
        nh.param("viz_jitter", gap_viz.viz_jitter, gap_viz.viz_jitter);
        nh.param("debug_viz", gap_viz.debug_viz, gap_viz.debug_viz);

        // Gap Association
        nh.param("assoc_thresh", gap_assoc.assoc_thresh, gap_assoc.assoc_thresh);

        // General Planning Mode Params
        nh.param("feasi_inflated", planning.feasi_inflated, planning.feasi_inflated);
        nh.param("projection_inflated", planning.projection_inflated, planning.projection_inflated);
        nh.param("planning_inflated", planning.planning_inflated, planning.planning_inflated);
        nh.param("holonomic", planning.holonomic, planning.holonomic);
        nh.param("full_fov", planning.full_fov, planning.full_fov);
        nh.param("projection_operator", planning.projection_operator, planning.projection_operator);
        nh.param("niGen_s", planning.niGen_s, planning.niGen_s);
        nh.param("num_feasi_check", planning.num_feasi_check, planning.num_feasi_check);
        nh.param("num_feasi_check", planning.far_feasible, planning.far_feasible);

        // Robot
        nh.param("r_inscr", rbt.r_inscr, rbt.r_inscr);
        nh.param("num_obsts", rbt.num_obsts, rbt.num_obsts);

    }
}