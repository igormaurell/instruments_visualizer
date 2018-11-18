#include "ros/ros.h"

#include "std_msgs/Header.h"

#include "instruments_visualizer/VisualizeManometer.h"

#include "AnalogMeterDetectorCLT.h"

std_msgs::Header state_header;

bool visualizeManometer(instruments_visualizer::VisualizeManometer::Request &req, instruments_visualizer::VisualizeManometer::Response &res)
{
    ROS_INFO("Reading manometer state...");
    
    instruments_visualizer::ManometerState &manometer_state = res.manometer_state;


    state_header.stamp = ros::Time::now();

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "manometer_visualizer_node");

    ros::NodeHandle nh;

    VisionSense::AnalogMeterDetectorCLT amd;

    ros::ServiceServer service = nh.advertiseService("/instruments_visualizer/visualize_manometer", );

    ros::spin();
    return 0;
}