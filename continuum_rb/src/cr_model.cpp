#include "continuum_rb/Continuum.h"
#include <iostream>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "core_node");
  ros::NodeHandle nh_;
  Continuum robot(8,4);
  // Parameter to decide whether position or kappa is given as input to the robot
  bool position = true;
  robot.setBasePose();
  robot.update();

  // Subscriber used to receive the robot position
  ros::Subscriber poseSub  = nh_.subscribe("/poseEE",10,&Continuum::pose_callback, &robot);

  if(position)
  {
    // Giving robot position as input
    while(ros::ok())
    {
      robot.setrbShape(0);
      robot.update();
      ros::spinOnce();
    }
  }
  else
  {
    // Giving kappa as input
    while(ros::ok())
      {
        for(double i=.5;i>=0.1;i=i-0.01)
        {
          robot.setrbShape(0,i);
          robot.update();
          ros::spinOnce();
        }
      }
  }
}