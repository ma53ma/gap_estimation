#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <ros/ros.h>
#include <math.h>
#include <gap_estimation/gap.h>
#include <gap_estimation/gap_estimator_config.h>
#include <vector>
#include <map>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

namespace gap_estimation {

    class Visualizer {
        public: 
            Visualizer() {};
            ~Visualizer() {};

            Visualizer(ros::NodeHandle& nh, const gap_estimation::GapEstimatorConfig& cfg);
            Visualizer& operator=(Visualizer other) {cfg_ = other.cfg_;};
            Visualizer(const Visualizer &t) {cfg_ = t.cfg_;};

        protected:
            const GapEstimatorConfig* cfg_;
    };

    class GapVisualizer : public Visualizer{
            using Visualizer::Visualizer;
        public: 

            GapVisualizer(ros::NodeHandle& nh, const gap_estimation::GapEstimatorConfig& cfg);
            void initialize(ros::NodeHandle& nh, const gap_estimation::GapEstimatorConfig& cfg);
            void drawGap(visualization_msgs::MarkerArray &, Gap g, std::string ns, bool initial);
            void drawGaps(std::vector<Gap> g, std::string ns);
            void drawGapsModels(std::vector<Gap> g);
            void drawGapModels(visualization_msgs::MarkerArray & model_arr, visualization_msgs::MarkerArray & gap_vel_arr, Gap g, std::string ns);


        private:
            std::map<std::string, std::vector<std_msgs::ColorRGBA>> colormap;
            ros::Publisher gaparc_publisher;
            ros::Publisher gapmodel_pos_publisher;
            ros::Publisher gapmodel_vel_publisher;
    };
}

#endif