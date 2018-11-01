#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "robotROSFeedbackControlExample";

// For Block robotROSFeedbackControlExample/Subscribe
SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_robotROSFeedbackControlExample_nav_msgs_Odometry> Sub_robotROSFeedbackControlExample_126;

// For Block robotROSFeedbackControlExample/Command Velocity Publisher/Publish2
SimulinkPublisher<geometry_msgs::Twist, SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Twist> Pub_robotROSFeedbackControlExample_128;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

