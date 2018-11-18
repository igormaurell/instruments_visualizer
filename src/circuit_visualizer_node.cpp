#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "circuit_visualizer_node");

    ros::NodeHandle nh;


    ros::spin();
    return 0;
}