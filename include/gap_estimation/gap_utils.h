#ifndef GAP_UTILS_H
#define GAP_UTILS_H

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <gap_estimation/gap.h>
#include <boost/shared_ptr.hpp>
#include <gap_estimation/gap_estimator_config.h>

namespace gap_estimation {

    class GapUtils 
    {
        public: 
            GapUtils();

            ~GapUtils();

            GapUtils(const gap_estimation::GapEstimatorConfig& cfg);

            GapUtils& operator=(gap_estimation::GapUtils other) {cfg_ = other.cfg_;};

            GapUtils(const GapUtils &t) {cfg_ = t.cfg_;};

            std::vector<Gap> hybridScanGap(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
                                                        geometry_msgs::PoseStamped final_goal_rbt);

            std::vector<Gap> mergeGapsOneGo(boost::shared_ptr<sensor_msgs::LaserScan const>, std::vector<Gap>&);

        private:
            const GapEstimatorConfig* cfg_;

    };
}

#endif