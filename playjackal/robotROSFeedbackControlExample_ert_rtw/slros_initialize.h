#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block robotROSFeedbackControlExample/Subscribe
extern SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_robotROSFeedbackControlExample_nav_msgs_Odometry> Sub_robotROSFeedbackControlExample_126;

// For Block robotROSFeedbackControlExample/Command Velocity Publisher/Publish2
extern SimulinkPublisher<geometry_msgs::Twist, SL_Bus_robotROSFeedbackControlExample_geometry_msgs_Twist> Pub_robotROSFeedbackControlExample_128;

void slros_node_init(int argc, char** argv);

#endif
