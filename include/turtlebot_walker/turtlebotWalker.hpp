#ifndef INCLUDE_TURTLEBOT_WALKER_HPP_
#define INCLUDE_TURTLEBOT_WALKER_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

class turtlebotWalker {
 private:
    ros:Nodehandle n;
    ros::Subscriber sub;
    ros::Publisher velPub;
    bool collision;
    geometry_msgs::Twist msg;
    
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void navigateBot();
  
 public:
    turtlebotWalker();
    ~turtlebotWalker();
 }
