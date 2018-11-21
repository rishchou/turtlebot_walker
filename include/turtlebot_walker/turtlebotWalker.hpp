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
 *  @file    turtlebotWalker.hpp
 *  @author  Rishabh Choudhary
 *  @copyright MIT License
 *  @date    11/19/2018
 *  @version 1.0
 *  
 *  @brief  ENPM808X : Assignment to implement walker algorithm for turtlebot
 *
 *  @section DESCRIPTION
 *
 *  This file defines the turtlebotWalker class definition. 
 *
 */

#ifndef INCLUDE_TURTLEBOT_WALKER_HPP_
#define INCLUDE_TURTLEBOT_WALKER_HPP_

// ROS headers
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>

/**
 *  @brief Class turtlebotWalker
 *
 *  The following class turtlebotWalker subscribes to laserScan data 
 *  and  publishes command velocity to illustrate a walker mechanism by turtlebot
 *  turtlebot  
 */
class turtlebotWalker {
 private:
    // ROS Node handle object
    ros::NodeHandle n;
    // ROS subscriber object
    ros::Subscriber sub;
    // ROS publisher object
    ros::Publisher velPub;
    // Boolean flag to detect collision 
    bool collision;
    // Message type to publish linear and angular velocities
    geometry_msgs::Twist msg;
    
  
 public:
   /**
    *   @brief  Constructor for turtlebotWalker class
    *           Initializes the object  
    *   @param  none
    * 
    *   @return void
    */
    turtlebotWalker();
   /**
    *   @brief  Destructor for turtlebotWalker class
    *            Destroys the object 
    *   @param  none
    *
    *   @return void
    */

    ~turtlebotWalker();
   /**
    *   @brief  Callback function for subscriber to process laserScan data
    *            
    *   @param  pointer to LaserScan mesage 
    *
    *   @return void
    */
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

   /**
    *   @brief  Function to detect obstable nearby
    *            
    *   @param  none
    *
    *   @return true if object nearby, false otherwise
    */

    bool detectObstacle();

   /**
    *   @brief function to move the bot around
    *            
    *   @param  none
    *
    *   @return void
    */
    void navigateBot();
 };

#endif
