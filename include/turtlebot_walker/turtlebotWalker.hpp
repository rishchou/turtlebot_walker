#ifndef INCLUDE_TURTLEBOT_WALKER_HPP_
#define INCLUDE_TURTLEBOT_WALKER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>

class turtlebotWalker {
 private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher velPub;
    bool collision;
    geometry_msgs::Twist msg;
    
  
 public:
    turtlebotWalker();
    ~turtlebotWalker();
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    bool detectObstacle();
    void navigateBot();
 };

#endif
