#include <ros/ros.h>
#include <gap_estimation/gap_estimation.h>

int main(int argc, char **argv)
{
  std::string name= "gap_estimation_node";
  ros::init(argc, argv, name);
  ros::start();
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  gap_estimation::Planner planner;

  ros::Subscriber laser_sub = nh.subscribe("/point_scan", 1, &gap_estimation::Planner::laserScanCB, &planner);
  ros::Subscriber pose_sub = nh.subscribe("/robot2/odom", 1, &gap_estimation::Planner::poseCB, &planner);

  ros::spin();
}
