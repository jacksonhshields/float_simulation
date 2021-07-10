/*
 * Bridge Topics
 * Purpose: Converts the topics from the simulation to the real
 * Subscribes:
 *  -
 * Publishes:
 * -
 * Services:
 * -
 * Outputs: Motor Commands
*/
#include <math.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"

#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"

class Bridger
{
public:
    Bridger()
    {
        depthPub_ = nh_.advertise<std_msgs::Float64>("depth", 1);
        depthStampedPub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("depth_stamped", 1);
        thruster0Pub_ = nh_.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("bffv1/thrusters/0/input", 1);
        thruster1Pub_ = nh_.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("bffv1/thrusters/1/input", 1);

        thrusterCmdSub_ = nh_.subscribe("thruster_cmd", 1, &Bridger::thrusterCmdCallback, this);
        poseSub_ = nh_.subscribe("bffv1/pose_gt", 1, &Bridger::poseCallback, this);

    }
    void thrusterCmdCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        uuv_gazebo_ros_plugins_msgs::FloatStamped tmsg0, tmsg1;
        tmsg0.header.seq = thruster_count_;
        tmsg0.header.stamp = ros::Time::now();
        tmsg0.header.frame_id = "/bffv1/thruster0";
        double scaled_cmd = msg->data * 1000.0;
        tmsg0.data = scaled_cmd;
        tmsg1 = tmsg0;
        tmsg1.header.frame_id = "/bffv1/thruster1";
        thruster0Pub_.publish(tmsg0);
        thruster1Pub_.publish(tmsg1);
        thruster_count_++;
    }

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double depth = -msg->pose.pose.position.z;
        std_msgs::Float64 depth_msg;
        depth_msg.data = depth;
        depthPub_.publish(depth_msg);

        geometry_msgs::Vector3Stamped depth_stamped_msg;
        depth_stamped_msg.vector.z = depth;
        depth_stamped_msg.header.stamp = ros::Time::now();
        depth_stamped_msg.header.seq = depth_count_;
        depth_stamped_msg.header.frame_id = "pressure_sensor";
        depthStampedPub_.publish(depth_stamped_msg);
    }



private:
    // The nodehandle
    ros::NodeHandle nh_;

    ros::Publisher depthPub_;
    ros::Publisher depthStampedPub_;
    ros::Publisher thruster0Pub_;
    ros::Publisher thruster1Pub_;


    // Subscribers
    ros::Subscriber poseSub_; // used for getting depth
    ros::Subscriber thrusterCmdSub_;

    // Sequence counts
    int thruster_count_;
    int depth_count_;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "bridger");

    Bridger bridger;

    ros::spin();

    return 0;
}