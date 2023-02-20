#include "ros/ros.h"
#include <iostream>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Point.h"

int main(int argc,char **argvc)
{
    ros::init(argc,argvc,"publishKappa");
    ros::NodeHandle nh;
    ros::Publisher kappa_publisher = nh.advertise<std_msgs::Float64>("/poseKP",10);
    ros::Rate rate(10);
    std_msgs::Float64 kappa;
    while(ros::ok())
    {
        std::cout<<"Enter the Curvature"<<std::endl;
        std::cin>>kappa.data;

        kappa_publisher.publish(kappa);
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}