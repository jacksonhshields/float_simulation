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
#include "sensor_msgs/Range.h"

class Bridger
{
public:
    Bridger()
    {


        tgtDepthPub_ = nh_.advertise<std_msgs::Float64>("depth_target", 1);
        thrusterPub_ = nh_.advertise<std_msgs::Float64>("thruster_cmd", 1);
        pidEnablePub_ = nh_.advertise<std_msgs::Bool>("pid_enable", 1);

        ros::ServiceServer thrusterCmdService_ = nh_.advertiseService("set_thruster_command", &Controller::thrusterCmdServiceCallback, this);
        ros::ServiceServer depthCmdService_ = nh_.advertiseService("set_depth_target", &Controller::depthCmdServiceCallback, this);
        ros::ServiceServer altitudeCmdService_ = nh_.advertiseService("set_altitude_target", &Controller::altitudeCmdServiceCallback, this);


        altitudeSub_ =  nh_.subscribe("ping", 1, &Controller::altitudeCallback, this);
        depthSub_ =  nh_.subscribe("depth", 1, &Controller::depthCallback, this);

        double control_freq = 10.0; // The control frequency - make a parameter

        ros::Timer timer = nh_.createTimer(ros::Duration(1/control_freq), &Controller::timerCallback, this);

    }

    // Control timer callback - publishes commands at this interval
    void timerCallback(const ros::TimerEvent& event)
    {
        // Check for the timeout
        if (ros::Time::now() - time0_ >= timeout_)
        {
            ROS_WARN_STREAM("Current command has timed out. Changing to 'thruster' mode with zero command");
            mode_ = THRUSTER;
            thruster_set_ = 0.0;
        }

        if (mode_ == THRUSTER)
        {
            // Disable the PID
            std_msgs::Bool disable_msg;
            disable_msg.data = false;
            pidEnablePub_.publish(disable_msg);
            // Set the thruster command
            std_msgs::Float64 cmd_msg;
            cmd_msg.data = thruster_set_;
            thrusterPub_.publish(cmd_msg);
        }
        else if (mode_ == DEPTH)
        {
            if (new_depth_ )
            {
                // Enable the PID
                std_msgs::Bool enable_msg;
                enable_msg.data = true;
                pidEnablePub_.publish(enable_msg);

                // Publish the depth target
                std_msgs::Float64 tgt_msg;
                tgt_msg.data = depth_set_;
                tgtDepthPub_.publish(tgt_msg);

                no_sensor_count_ = 0;
            }
            else
            {
                // Reaches here if sensor data hasn't been received
                no_sensor_count_ ++;
                // If more than 5 loops have elapsed, and sensor data hasn't been received, turn motors off.
                if (no_sensor_count_ > 5)
                {
                    ROS_WARN_STREAM("Sensor readings not coming in - last depth=" << ros::Time::now() - last_depth_time_ << ", last altitude=" << ros::Time::now() - last_altitude_time_);
                    ROS_WARN_STREAM("Settings to 'thruster' mode with zero command");
                    mode_ = THRUSTER;
                    thruster_set_ = 0.0;
                }
            }
        }
        else if (mode_ == ALTITUDE)
        {
            if (new_depth_ && new_altitude_)
            {
                // Enable the PID
                std_msgs::Bool enable_msg;
                enable_msg.data = true;
                pidEnablePub_.publish(enable_msg);

                // Find the target depth
                double tgt_depth = calc_target_depth(depth_, altitude_set_, altitude_set_);
                std_msgs::Float64 tgt_msg;
                tgt_msg.data = tgt_depth;
                tgtDepthPub_.publish(tgt_msg);

                no_sensor_count_ = 0;
            }
            else
            {
                // Reaches here if sensor data hasn't been received
                no_sensor_count_ ++;
                // If more than 5 loops have elapsed, and sensor data hasn't been received, turn motors off.
                if (no_sensor_count_ > 5)
                {
                    ROS_WARN_STREAM("Sensor readings not coming in - last depth=" << ros::Time::now() - last_depth_time_ << ", last altitude=" << ros::Time::now() - last_altitude_time_);
                    ROS_WARN_STREAM("Settings to 'thruster' mode with zero command");
                    mode_ = THRUSTER;
                    thruster_set_ = 0.0;
                }
            }
        }
    }

    // Receives the new altitude estimate
    void altitudeCallback(const sensor_msgs::Range::ConstPtr& msg)
    {
        altitude_ = msg->range;
        new_depth_ = true;
        mode_ = ALTITUDE;
        last_altitude_time_ = ros::Time::now();
    }

    // Receives the new depth estimate
    void depthCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        depth_ = msg->data;
        new_depth_ = true;
        mode_ = DEPTH;
        last_depth_time_ = ros::Time::now();
    }

    bool thrusterCmdServiceCallback(float_control::SetThrusterCommand::Request &req, float_control::SetThrusterCommand::Response &res)
    {
        // Retrieve the commands
        timeout_ = ros::Duration(req.timeout);
        time0_ = ros::Time::now();
        thruster_set_ = req.thruster_command;

        // Set the response
        res.success = true;
        return true;
    }

    bool depthCmdServiceCallback(float_control::SetDepthTarget::Request &req, float_control::SetDepthTarget::Response &res)
    {
        // Retrieve the commands
        timeout_ = ros::Duration(req.timeout);
        time0_ = ros::Time::now();
        depth_set_ = req.depth_target;

        // Set the response
        res.success = true;
        return true;
    }

    bool altitudeCmdServiceCallback(float_control::SetAltitudeTarget::Request &req, float_control::SetAltitudeTarget::Response &res)
    {
        // Retrieve the commands
        timeout_ = ros::Duration(req.timeout);
        time0_ = ros::Time::now();
        altitude_set_ = req.altitude_target;
        // Set the response
        res.success = true;
        return true;
    }

    // Calc target depth
    // cd = current_depth, ca=current_altitude, td=target_depth, ta=target_altitude
    double calc_target_depth(double cd, double ca, double ta)
    {
        // current_depth + current_altitude = target_depth + target_altitude
        // target_depth = current_depth + current_altitude - target_altitude
        double td;
        td = cd + ca - ta;
        return td;
    }


private:
    // The nodehandle
    ros::NodeHandle nh_;

    // Mode selection
    enum mode_select mode_;

    // Timeout
    ros::Duration timeout_; // Maximum time that can elapse before switching thrusters off
    ros::Time time0_; // Time of the last command

    // Variables to hold the set commands
    double thruster_set_;
    double depth_set_;
    double altitude_set_;

    // Variables to hold values of altitude and depth
    double depth_;
    double altitude_;

    // Flags to indicate when we have information
    bool depth_received_;
    bool altitude_received_;

    // Flags to determine when we have new information
    bool new_depth_;
    bool new_altitude_;

    // Variables holding the time information was last received
    ros::Time last_depth_time_;
    ros::Time last_altitude_time_;

    // Counter for warning the user when sensors stop receiving
    int no_sensor_count_;

    // Services
    ros::ServiceServer thruster_cmd_service_;
    ros::ServiceServer depth_cmd_service_;
    ros::ServiceServer altitude_cmd_service_;

    // Publishers
    ros::Publisher tgtDepthPub_;
    ros::Publisher thrusterPub_;
    ros::Publisher pidEnablePub_;

    // Subscribers
    ros::Subscriber altitudeSub_;
    ros::Subscriber depthSub_;


};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "bridger");

    Bridger bridger();

    ros::spin();

    return 0;
}