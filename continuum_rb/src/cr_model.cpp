#include "continuum_rb/Continuum.h"
#include <iostream>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "core_node");
  ros::NodeHandle nh_;
  bool position = 0;
  Continuum robot(8,4);
  // Parameter to decide whether position or kappa is given as input to the robot
  robot.setBasePose();
  robot.update();

if(position)
{
  ros::Subscriber poseSub  = nh_.subscribe("/poseEE",10,&Continuum::pose_callback, &robot);

  while(ros::ok())
  {
    robot.setrbShape(0);
    robot.update();
    ros::spinOnce();
  }
}
else
{
  ros::Subscriber kappaSub  = nh_.subscribe("/poseKP",10,&Continuum::kappa_callback, &robot);

  while(ros::ok())
  {
    robot.setkpShape(0);
    robot.update();
    ros::spinOnce();
  }
}
  // Subscriber used to receive the robot position
  
    
}