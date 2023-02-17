#include "continuum_rb/Continuum.h"
#include <iostream>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "core_node");
  ros::NodeHandle nh_;
  Continuum robot(8,4);
  // Parameter to decide whether position or kappa is given as input to the robot
  robot.setBasePose();
  robot.update();

  // Subscriber used to receive the robot position
  ros::Subscriber poseSub  = nh_.subscribe("/poseEE",10,&Continuum::pose_callback, &robot);

  while(ros::ok())
  {
    robot.setrbShape(0);
    robot.update();
    ros::spinOnce();
  }
    
}