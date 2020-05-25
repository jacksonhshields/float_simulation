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

    }



private:
    // The nodehandle
    ros::NodeHandle nh_;

    ros::Publisher depthPub_;

    // Subscribers
    ros::Subscriber poseSub_;


};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "bridger");

    Bridger bridger();

    ros::spin();

    return 0;
}