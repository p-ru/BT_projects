#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "trajGen";

// For Block trajGen/Subscribe
SimulinkSubscriber<geometry_msgs::Point, SL_Bus_trajGen_geometry_msgs_Point> Sub_trajGen_180;

// For Block trajGen/position reading
SimulinkSubscriber<nav_msgs::Odometry, SL_Bus_trajGen_nav_msgs_Odometry> Sub_trajGen_56;

// For Block trajGen/Publish
SimulinkPublisher<geometry_msgs::Twist, SL_Bus_trajGen_geometry_msgs_Twist> Pub_trajGen_43;

// For Block trajGen/Get Parameter
SimulinkParameterGetter<real64_T, double> ParamGet_trajGen_152;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

