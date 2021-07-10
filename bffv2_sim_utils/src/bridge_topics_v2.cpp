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

#include "float_sensor_msgs/Depth.h"
#include "float_sensor_msgs/PingRange.h"

class Bridger
{
public:
    Bridger()
    {
        depthRawPub_ = nh_.advertise<std_msgs::Float64>("pressure_sensor/depth_raw", 1);
        depthPub_ = nh_.advertise<float_sensor_msgs::Depth>("pressure_sensor/depth", 1);
//        sonarPub_ = nh_.advertise<sensor_msgs::Range>("sonar_sensor/range", 1);
        thruster0Pub_ = nh_.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("bffv2/thrusters/0/input", 1);
        thruster1Pub_ = nh_.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("bffv2/thrusters/1/input", 1);

	float density_;
	float p0_;
	ros::param::param<float>("~density_pa", density_, 10000);
        thrusterCmdSub_ = nh_.subscribe("thruster_cmd", 1, &Bridger::thrusterCmdCallback, this);
        poseSub_ = nh_.subscribe("bffv2/pose_gt", 1, &Bridger::poseCallback, this);
//        sonarSub_ = nh_.subscribe("bffv2/altimeter", 1, &Bridger::sonarCallback, this);

    }
    void thrusterCmdCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        uuv_gazebo_ros_plugins_msgs::FloatStamped tmsg0, tmsg1;
        tmsg0.header.seq = thruster_count_;
        tmsg0.header.stamp = ros::Time::now();
        tmsg0.header.frame_id = "/bffv2/thruster0";
        double scaled_cmd = msg->data * 1000.0;
        tmsg0.data = scaled_cmd;
        tmsg1 = tmsg0;
        tmsg1.header.frame_id = "/bffv2/thruster1";
        thruster0Pub_.publish(tmsg0);
        thruster1Pub_.publish(tmsg1);
        thruster_count_++;
    }
    float depth2pressure(float depth)
    {
	    float pressure = (density_ * depth)+p0_;
	    return pressure;
    }

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double depth = -msg->pose.pose.position.z;
        std_msgs::Float64 depth_raw_msg;
        depth_raw_msg.data = depth;
        depthRawPub_.publish(depth_raw_msg);
	float depth_f = (float) depth;
	float pressure;
	pressure = Bridger::depth2pressure(depth_f);
	float_sensor_msgs::Depth depth_msg;
	depth_msg.header.stamp = ros::Time::now();
	depth_msg.header.seq = depth_count_;
        depth_msg.header.frame_id = "pressure_sensor";
        depth_msg.depth = depth_f;
	depth_msg.pressure = pressure; 
        depthPub_.publish(depth_msg);
	depth_count_++;
    }
/*
    void sonarCallback(const sensor_msgs::Range::ConstPtr& msg)
    {
	    sensor_msgs::Range sonar_msg;
	    sonar_msg.range = (float) msg->range;
	    sonar_msg.header = msg->header;
	    sonar_msg.max_range = 30.0;
	    sonar_msg.min_range = 0.0;
	    sonarPub_.publish(sonar_msg);
    }
*/


private:
    // The nodehandle
    ros::NodeHandle nh_;

    ros::Publisher depthPub_;
    ros::Publisher depthRawPub_;
//    ros::Publisher sonarPub_;
    ros::Publisher thruster0Pub_;
    ros::Publisher thruster1Pub_;


    // Subscribers
    ros::Subscriber poseSub_; // used for getting depth
    ros::Subscriber thrusterCmdSub_;
//    ros::Subscriber sonarSub_;

    // Sequence counts
    int thruster_count_;
    int depth_count_;
    float p0_;
    float density_;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "bridger");

    Bridger bridger;

    ros::spin();

    return 0;
}
