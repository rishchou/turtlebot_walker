/**
 *  MIT License
 *
 *  Copyright (c) 2018 Rishabh Choudhary
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @file    turtlebotWalker.cpp
 *  @author  Rishabh Choudhary
 *  @copyright MIT License
 *  @date    11/19/2018
 *  @version 1.0
 *  
 *  @brief  ENPM808X : Assignment to implement walker algorithm for turtlebot
 *
 *  @section DESCRIPTION
 *
 *  This file defines the turtlebotWalker class method implementation.
 *  The class contains laserscan callback and navigate methods which allows
 *  the turtlebot to navigate and subscribe to the laserscan data to detect
 *  obstacles nearby and avoid collisions. Hence the turtlebot follows a 
 *  simple walker algorithm like a roomba robot. 
 *
 */

// CPP header
#include <iostream>
// turtlebotWalker class header
#include "../include/turtlebot_walker/turtlebotWalker.hpp" 

turtlebotWalker::turtlebotWalker() {
    ROS_INFO_STREAM("turtlebot_walker node initialized");
    // Initialize class params
    collision = false;
    // advertise the publisher topic with rosmaster
    velPub = n.advertise <geometry_msgs::Twist> ("/cmd_vel_mux/input/navi", 1000);
    // SUbscribe to the laserscan topic 
    sub = n.subscribe("/scan", 500, &turtlebotWalker::laserScanCallback, this);
    //define the initial velocities
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    // publish the initial velocities
    velPub.publish(msg);
}

turtlebotWalker::~turtlebotWalker() {
    //stop the turtlebot before exiting
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    // publish the  final velocities 
    velPub.publish(msg);
}


void turtlebotWalker::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Loop though the laserscan mesages to check collision 
    for(int i = 0; i < msg->ranges.size(); ++i) {
    // Minimum threshold  distance for collision = 0.60
        if(msg->ranges[i] < 0.60) {
            collision = true;
            return;
        } 
    } 
    collision = false;
    return;
}

bool turtlebotWalker::detectObstacle() {
    // return the collision flag
    return collision;
}

void turtlebotWalker::navigateBot() {
    // Initialize the publisher freq
    ros::Rate loop_rate(10);
    // Implement till ros is running good
    while (ros::ok()) {
        // If obstacle is detected, turn the turtlebot 
        if(detectObstacle()) {
            ROS_WARN_STREAM("Obstacle ahead, turning bot");
            // Stop the forward motion 
            msg.linear.x = 0.0;
            // Rotate the bot
            msg.angular.z = -1.0;
        } else {
            ROS_INFO_STREAM("No obstacle ahead, moving straight");
            // If no obstacle, keep moving forward
            msg.linear.x = 0.2;
            msg.angular.z = 0.0;
        }
        // Publish the updated velocities	 
        velPub.publish(msg);

        ros::spinOnce();
    
        loop_rate.sleep();
    }
}
