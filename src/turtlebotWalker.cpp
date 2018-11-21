#include <iostream>
#include "../include/turtlebot_walker/turtlebotWalker.hpp" 

turtlebotWalker::turtlebotWalker() {
    collision = false;
    velPub = n.advertise <geometry_msgs::Twist> ("/cmd_vel_mux/input/navi", 1000);
    sub = n.subscribe("/scan", 500, &turtlebotWalker::laserScanCallback, this);
    //define the initial velocities
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
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
    velPub.publish(msg);
}


void turtlebotWalker::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for(int i = 0; i < msg->ranges.size(); ++i) {
        if(msg->ranges[i] < 0.60) {
            collision = true;
            return;
        } 
    }
    
    collision = false;
    
    return;
}

bool turtlebotWalker::detectObstacle() {
    return collision;
}

void turtlebotWalker::navigateBot() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if(detectObstacle()) {
            msg.linear.x = 0.0;
            msg.angular.z = -1.0;
        } else {
            msg.linear.x = 0.2;
            msg.angular.z = 0.0;
        }

        velPub.publish(msg);

        ros::spinOnce();
    
        loop_rate.sleep();
    }
}
