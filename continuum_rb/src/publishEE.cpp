#include "ros/ros.h"
#include <iostream>
#include "std_msgs/Int32.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Point.h"

int main(int argc,char **argvc)
{
    ros::init(argc,argvc,"publishEE");
    ros::NodeHandle nh;
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::Point>("/poseEE",10);
    ros::Rate rate(10);
    geometry_msgs::Point pt;
    while(ros::ok())
    {
        std::cout<<"Enter the X Coordinate"<<std::endl;
        std::cin>>pt.x;

        std::cout<<"Enter the Y Coordinate"<<std::endl;
        std::cin>>pt.y;

        std::cout<<"Enter the Z Coordinate"<<std::endl;
        std::cin>>pt.z;

        pose_publisher.publish(pt);
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}