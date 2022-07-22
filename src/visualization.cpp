#include <gap_estimation/visualization.h>

namespace gap_estimation {

    GapVisualizer::GapVisualizer(ros::NodeHandle& nh, const gap_estimation::GapEstimatorConfig& cfg) {
        initialize(nh, cfg);
    }

    void GapVisualizer::initialize(ros::NodeHandle& nh, const gap_estimation::GapEstimatorConfig& cfg) {
        cfg_ = &cfg;
        gaparc_publisher = nh.advertise<visualization_msgs::MarkerArray>("pg_arcs", 1000);

        gapmodel_pos_publisher = nh.advertise<visualization_msgs::MarkerArray>("dg_model_pos", 10);
        gapmodel_vel_publisher = nh.advertise<visualization_msgs::MarkerArray>("dg_model_vel", 10);

        std_msgs::ColorRGBA std_color;
        std::vector<std_msgs::ColorRGBA> raw;
        std::vector<std_msgs::ColorRGBA> simp_expanding;
        std::vector<std_msgs::ColorRGBA> simp_closing;
        std::vector<std_msgs::ColorRGBA> simp_translating;
        std::vector<std_msgs::ColorRGBA> manip_expanding;
        std::vector<std_msgs::ColorRGBA> manip_closing;
        std::vector<std_msgs::ColorRGBA> manip_translating;
        std::vector<std_msgs::ColorRGBA> gap_model;

        std::vector<std_msgs::ColorRGBA> raw_initial;

        std::vector<std_msgs::ColorRGBA> simp_initial;
        std::vector<std_msgs::ColorRGBA> simp_terminal;
        std::vector<std_msgs::ColorRGBA> manip_initial;
        std::vector<std_msgs::ColorRGBA> manip_terminal;
        std::vector<std_msgs::ColorRGBA> reachable_gap_centers;
        std::vector<std_msgs::ColorRGBA> gap_splines;

        // Raw Therefore Alpha halved
        // RAW: RED
        // RAW RADIAL

        // RAW
        std_color.a = 1.0;
        std_color.r = 0.0; //std_color.r = 0.6;
        std_color.g = 0.0; //std_color.g = 0.2;
        std_color.b = 0.0; //std_color.b = 0.1;
        raw.push_back(std_color);
        raw.push_back(std_color);
        
        // SIMPLIFIED EXPANDING
        std_color.a = 0.5;
        std_color.r = 1.0; //std_color.r = 1;
        std_color.g = 0.0; //std_color.g = 0.9;
        std_color.b = 0.0; //std_color.b = 0.3;
        simp_expanding.push_back(std_color);
        simp_expanding.push_back(std_color);
        // SIMPLIFIED CLOSING
        std_color.r = 0.0; //std_color.r = 0.4;
        std_color.g = 1.0; //std_color.g = 0;
        std_color.b = 0.0; //std_color.b = 0.9;
        simp_closing.push_back(std_color);
        simp_closing.push_back(std_color);
        // SIMPLIFIED TRANSLATING
        std_color.r = 0.0; //std_color.r = 0.4;
        std_color.g = 0.0; //std_color.g = 0;
        std_color.b = 1.0; //std_color.b = 0.9;
        simp_translating.push_back(std_color);
        simp_translating.push_back(std_color);

        // MANIPULATED EXPANDING
        std_color.a = 1.0;
        std_color.r = 1.0; //std_color.r = 1;
        std_color.g = 0.0; //std_color.g = 0.9;
        std_color.b = 0.0; //std_color.b = 0.3;
        manip_expanding.push_back(std_color);
        manip_expanding.push_back(std_color);
        // MANIPULATED CLOSING
        std_color.r = 0.0; //std_color.r = 0.4;
        std_color.g = 1.0; //std_color.g = 0;
        std_color.b = 0.0; //std_color.b = 0.9;
        manip_closing.push_back(std_color);
        manip_closing.push_back(std_color);
        // MANIPULATED TRANSLATING
        std_color.r = 0.0; //std_color.r = 0.4;
        std_color.g = 0.0; //std_color.g = 0;
        std_color.b = 1.0; //std_color.b = 0.9;
        manip_translating.push_back(std_color);
        manip_translating.push_back(std_color);

        std_color.a = 1.0;
        std_color.r = 0.0; //std_color.r = 0.6;
        std_color.g = 0.0; //std_color.g = 0.2;
        std_color.b = 0.0; //std_color.b = 0.1;
        raw_initial.push_back(std_color);
        raw_initial.push_back(std_color);
        
        
        std_color.a = 1.0;
        std_color.r = 1.0;
        std_color.g = 0.0;
        std_color.b = 0.0; 
        simp_initial.push_back(std_color);
        simp_initial.push_back(std_color);

        std_color.a = 0.50;
        std_color.r = 1.0;
        std_color.g = 0.0; 
        std_color.b = 0.0; 
        simp_terminal.push_back(std_color);
        simp_terminal.push_back(std_color);

        std_color.a = 1.0;
        std_color.r = 0.0; 
        std_color.g = 1.0;
        std_color.b = 0.0;
        manip_initial.push_back(std_color);
        manip_initial.push_back(std_color);

        std_color.a = 0.50;
        std_color.r = 0.0; 
        std_color.g = 1.0; 
        std_color.b = 0.0; 
        manip_terminal.push_back(std_color);
        manip_terminal.push_back(std_color);

        // MODELS: PURPLE
        std_color.r = 1;
        std_color.g = 0;
        std_color.b = 1;
        gap_model.push_back(std_color);
        gap_model.push_back(std_color);

        std_color.r = 1;
        std_color.g = 1;
        std_color.b = 0;
        reachable_gap_centers.push_back(std_color);
        reachable_gap_centers.push_back(std_color);

        std_color.r = 0.8;
        std_color.g = 1;
        std_color.b = 0;
        gap_splines.push_back(std_color);
        gap_splines.push_back(std_color);        

        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("raw", raw));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("simp_expanding", simp_expanding));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("simp_closing", simp_closing));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("simp_translating", simp_translating));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("manip_expanding", manip_expanding));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("manip_closing", manip_closing));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("manip_translating", manip_translating));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("raw_initial", raw_initial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("simp_initial", simp_initial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("simp_terminal", simp_terminal));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("manip_initial", manip_initial));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("manip_terminal", manip_terminal));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("reachable_gap_centers", reachable_gap_centers));
        colormap.insert(std::pair<std::string, std::vector<std_msgs::ColorRGBA>>("gap_splines", gap_splines));

    }

    void GapVisualizer::drawGap(visualization_msgs::MarkerArray & vis_arr, gap_estimation::Gap g, std::string ns, bool initial) {
        // ROS_INFO_STREAM(g._left_idx << ", " << g._ldist << ", " << g._right_idx << ", " << g._rdist << ", " << g._frame);
        if (!cfg_->gap_viz.debug_viz) return;
        //ROS_INFO_STREAM("in draw gap");
        int viz_offset = 0;
        double viz_jitter = cfg_->gap_viz.viz_jitter;
        
        if (viz_jitter > 0 && g.isAxial(initial)) {
            viz_offset = g.isRightType(initial) ? -2 : 2;
        }

        int lidx = initial ? g.RIdx() : g.term_RIdx();
        int ridx = initial ? g.LIdx() : g.term_LIdx();
        float ldist = initial ? g.RDist() : g.term_RDist();
        float rdist = initial ? g.LDist() : g.term_LDist();

        //ROS_INFO_STREAM("lidx: " << lidx << ", ldist: " << ldist << ", ridx: " << ridx << ", rdist: " << rdist);
        int gap_idx_size = (ridx - lidx);
        if (gap_idx_size < 0) {
            gap_idx_size += 2*g.half_scan; // taking off int casting here
        }

        int num_segments = gap_idx_size / cfg_->gap_viz.min_resoln + 1;
        float dist_step = (rdist - ldist) / num_segments;
        int sub_gap_lidx = lidx + viz_offset;
        float sub_gap_ldist = ldist;

        visualization_msgs::Marker this_marker;
        this_marker.header.frame_id = g._frame;
        this_marker.header.stamp = ros::Time();
        this_marker.ns = ns;
        this_marker.type = visualization_msgs::Marker::LINE_STRIP;
        this_marker.action = visualization_msgs::Marker::ADD;

        // std::cout << "ns coming in: " << ns << std::endl;
        std::string local_ns = ns;
        // for compare, 0 is true?
        
        float z_val = (ns == "raw") ? 0.0001 : 0.001;
        /*
        if (ns.compare("raw") != 0) {
            if (g.getCategory().compare("expanding") == 0) {
                local_ns.append("_expanding");
            } else if (g.getCategory().compare("closing") == 0) {
                local_ns.append("_closing");
            } else {
                local_ns.append("_translating");
            }
        }
        */
        if (initial) {
            local_ns.append("_initial");
        } else {
            local_ns.append("_terminal");
        }

        // std::cout << "gap category: " << g.getCategory() << std::endl;
        //ROS_INFO_STREAM("ultimate local ns: " << local_ns);
        auto color_value = colormap.find(local_ns);
        if (color_value == colormap.end()) {
            ROS_FATAL_STREAM("Visualization Color not found, return without drawing");
            return;
        }

        this_marker.colors = color_value->second;
        double thickness = cfg_->gap_viz.fig_gen ? 0.05 : 0.01;
        this_marker.scale.x = thickness;
        this_marker.scale.y = 0.0001;
        this_marker.scale.z = 0.0001;
        //bool finNamespace = (ns.compare("fin") == 0);

        geometry_msgs::Point linel;
        linel.z = z_val;
        geometry_msgs::Point liner;
        liner.z = z_val;
        std::vector<geometry_msgs::Point> lines;

        /*
        if (finNamespace) {
            this_marker.colors.at(0).a = 1;
            this_marker.colors.at(1).a = 1;
            linel.z = 0.1;
            liner.z = 0.1;
        }
        */

        int id = (int) vis_arr.markers.size();

        for (int i = 0; i < num_segments - 1; i++)
        {
            lines.clear();
            linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            sub_gap_lidx = (sub_gap_lidx + cfg_->gap_viz.min_resoln) % int(2*g.half_scan);
            sub_gap_ldist += dist_step;
            liner.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            liner.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
            lines.push_back(linel);
            lines.push_back(liner);

            this_marker.points = lines;
            this_marker.id = id++;
            this_marker.lifetime = ros::Duration(100.0);

            vis_arr.markers.push_back(this_marker);
        }

        // close the last
        // this_marker.scale.x = 10*thickness;
        lines.clear();
        linel.x = (sub_gap_ldist + viz_jitter) * cos(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        linel.y = (sub_gap_ldist + viz_jitter) * sin(-( (float) g.half_scan - sub_gap_lidx) / g.half_scan * M_PI);
        liner.x = (rdist + viz_jitter) * cos(-( (float) g.half_scan - ridx) / g.half_scan * M_PI);
        liner.y = (rdist + viz_jitter) * sin(-( (float) g.half_scan - ridx) / g.half_scan * M_PI);
        lines.push_back(linel);
        lines.push_back(liner);
        this_marker.points = lines;
        this_marker.id = id++;
        this_marker.lifetime = ros::Duration(100.0);
        vis_arr.markers.push_back(this_marker);
    }

    void GapVisualizer::drawGaps(std::vector<gap_estimation::Gap> g, std::string ns) {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = ns;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        gaparc_publisher.publish(clear_arr);

        visualization_msgs::MarkerArray vis_arr;
        for (auto gap : g) {
            drawGap(vis_arr, gap, ns, true);
            
            /*
            if (ns.compare("raw") != 0) {
                drawGap(vis_arr, gap, ns, false);
            }
            */
        }
        gaparc_publisher.publish(vis_arr);
    }

    void GapVisualizer::drawGapsModels(std::vector<gap_estimation::Gap> g) {
        if (!cfg_->gap_viz.debug_viz) return;

        // First, clearing topic.
        visualization_msgs::MarkerArray clear_arr;
        visualization_msgs::Marker clear_marker;
        clear_marker.id = 0;
        clear_marker.ns = "gap_models";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_arr.markers.push_back(clear_marker);
        gapmodel_pos_publisher.publish(clear_arr);
        gapmodel_vel_publisher.publish(clear_arr);

        visualization_msgs::MarkerArray gap_pos_arr;
        visualization_msgs::MarkerArray gap_vel_arr;
        for (auto gap : g) {
            drawGapModels(gap_pos_arr, gap_vel_arr, gap, "gap_models");
        }
        gapmodel_pos_publisher.publish(gap_pos_arr);
        gapmodel_vel_publisher.publish(gap_vel_arr);
    }

    void GapVisualizer::drawGapModels(visualization_msgs::MarkerArray & model_arr, visualization_msgs::MarkerArray & gap_vel_arr, gap_estimation::Gap g, std::string ns) {
        int model_id = (int) model_arr.markers.size();
        visualization_msgs::Marker right_model_pt;
        // VISUALIZING THE GAP-ONLY DYNAMICS (ADDING THE EGO VELOCITY)
        
        // std::cout << "model frame: " << g._frame << std::endl;
        right_model_pt.header.frame_id = g._frame;
        right_model_pt.header.stamp = ros::Time();
        right_model_pt.ns = ns;
        right_model_pt.id = model_id++;
        right_model_pt.type = visualization_msgs::Marker::CYLINDER;
        right_model_pt.action = visualization_msgs::Marker::ADD;
        right_model_pt.pose.position.x = g.right_model->get_cartesian_state()[0];
        right_model_pt.pose.position.y = g.right_model->get_cartesian_state()[1];
        right_model_pt.pose.position.z = 0.0001;
        //std::cout << "left point: " << right_model_pt.pose.position.x << ", " << right_model_pt.pose.position.y << std::endl;
        right_model_pt.pose.orientation.w = 1.0;
        right_model_pt.scale.x = 0.1;
        right_model_pt.scale.y = 0.1;
        right_model_pt.scale.z = 0.000001;
        right_model_pt.color.a = 1.0;
        right_model_pt.color.r = 1.0;
        right_model_pt.color.b = 1.0;
        right_model_pt.lifetime = ros::Duration(100.0);
        model_arr.markers.push_back(right_model_pt);

        visualization_msgs::Marker left_model_pt;
        // std::cout << "model frame: " << g._frame << std::endl;
        left_model_pt.header.frame_id = g._frame;
        left_model_pt.header.stamp = ros::Time();
        left_model_pt.ns = ns;
        left_model_pt.id = model_id++;
        left_model_pt.type = visualization_msgs::Marker::CYLINDER;
        left_model_pt.action = visualization_msgs::Marker::ADD;
        left_model_pt.pose.position.x = g.left_model->get_cartesian_state()[0];
        left_model_pt.pose.position.y = g.left_model->get_cartesian_state()[1];
        left_model_pt.pose.position.z = 0.0001;
        // std::cout << "right point: " << left_model_pt.pose.position.x << ", " << left_model_pt.pose.position.y << std::endl;
        left_model_pt.pose.orientation.w = 1.0;
        left_model_pt.scale.x = 0.1;
        left_model_pt.scale.y = 0.1;
        left_model_pt.scale.z = 0.000001;
        left_model_pt.color.a = 1.0;
        left_model_pt.color.r = 1.0;
        left_model_pt.color.b = 1.0;
        left_model_pt.lifetime = ros::Duration(100.0);
        model_arr.markers.push_back(left_model_pt);

        visualization_msgs::Marker right_model_vel_pt;
        // gap_vel_arr.markers.push_back(right_model_pt);
        right_model_vel_pt.header.frame_id = g._frame;
        right_model_vel_pt.header.stamp = ros::Time();
        right_model_vel_pt.ns = ns;
        right_model_vel_pt.id = model_id++;
        right_model_vel_pt.type = visualization_msgs::Marker::ARROW;
        right_model_vel_pt.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point right_vel_pt;
        right_vel_pt.x = right_model_pt.pose.position.x;
        right_vel_pt.y = right_model_pt.pose.position.y;
        right_vel_pt.z = 0.000001;
        right_model_vel_pt.points.push_back(right_vel_pt);
        Eigen::Vector2d right_vel(g.right_model->get_cartesian_state()[2] + g.right_model->get_v_ego()[0],
                                    g.right_model->get_cartesian_state()[3] + g.right_model->get_v_ego()[1]);
        //std::cout << "visualizing left gap only vel: " << right_vel_pt[0] << ", " << right_vel_pt[1] << std::endl;
        right_vel_pt.x = right_model_pt.pose.position.x + right_vel[0];
        right_vel_pt.y = right_model_pt.pose.position.y + right_vel[1];
        right_model_vel_pt.points.push_back(right_vel_pt);
        right_model_vel_pt.scale.x = 0.1;
        right_model_vel_pt.scale.y = 0.1;
        right_model_vel_pt.scale.z = 0.1;
        right_model_vel_pt.color.a = 1.0;
        right_model_vel_pt.color.r = 1.0;
        right_model_vel_pt.color.b = 1.0;
        right_model_vel_pt.lifetime = ros::Duration(100.0);
        gap_vel_arr.markers.push_back(right_model_vel_pt);
        // std::cout << "left velocity end point: " << right_vel_pt.x << ", " << right_vel_pt.y << std::endl;

        visualization_msgs::Marker left_model_vel_pt;
        left_model_vel_pt.header.frame_id = g._frame;
        left_model_vel_pt.header.stamp = ros::Time();
        left_model_vel_pt.ns = ns;
        left_model_vel_pt.id = model_id++;
        left_model_vel_pt.type = visualization_msgs::Marker::ARROW;
        left_model_vel_pt.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point left_vel_pt;
        left_vel_pt.x = left_model_pt.pose.position.x;
        left_vel_pt.y = left_model_pt.pose.position.y;
        left_vel_pt.z = 0.000001;
        left_model_vel_pt.points.push_back(left_vel_pt);
        Eigen::Vector2d left_vel(g.left_model->get_cartesian_state()[2] + g.left_model->get_v_ego()[0],
                                    g.left_model->get_cartesian_state()[3] + g.left_model->get_v_ego()[1]);
        // std::cout << "visualizing right gap only vel: " << left_vel_pt[0] << ", " << left_vel_pt[1] << std::endl;
        left_vel_pt.x = left_model_pt.pose.position.x + left_vel[0];
        left_vel_pt.y = left_model_pt.pose.position.y + left_vel[1];
        left_model_vel_pt.points.push_back(left_vel_pt);
        left_model_vel_pt.scale.x = 0.1;
        left_model_vel_pt.scale.y = 0.1;
        left_model_vel_pt.scale.z = 0.1;
        left_model_vel_pt.color.a = 1.0;
        left_model_vel_pt.color.r = 1.0;
        left_model_vel_pt.color.b = 1.0;
        left_model_vel_pt.lifetime = ros::Duration(100.0);
        gap_vel_arr.markers.push_back(left_model_vel_pt);
        // std::cout << "right velocity end point: " << right_vel_pt.x << ", " << right_vel_pt.y << std::endl;
    }
}