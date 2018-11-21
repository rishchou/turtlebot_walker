#include <iostream>
 #include "turtlebot_walker/turtlebotWalker.hpp" 

turtlebotWalker::turtlebotWalker(ros::Nodehandle &n) {
    collision = false;
    velPub = n.advertise <geometry_msgs::Twist> ("/mobile_base/commands/velocity", 100);
    sub = n.subscribe("scan", 50, &turtlebotWalker::laserScanCallback, this);
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


void turtlebot::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for(auto i : msg->ranges) {
        if(i < 1.0) {
            collision = true;
        } else {
            collision = false;
        }
    }
    return;
}

void navigateBot() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        if(collision) {
            msg.linear.x = 0.0;
            msg.angular.z = 0.5;
        } else {
            msg.angular.z = 0.0;
            msg.linear.x = 0.1;
        }

        velPub.publish(msg);

        ros::spinOnce();
    
        loop_rate.sleep();
    }
}
