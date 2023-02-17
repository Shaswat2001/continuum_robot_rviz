#include "continuum_rb/Continuum.h"
#include <iostream>

geometry_msgs::Point pt;
class Listener
{
  public:
    void pose_callback(const geometry_msgs::Point msg);
    
};

void Listener::pose_callback(const geometry_msgs::Point msg)
{
    pt = msg;
}
int main( int argc, char** argv )
{
  ros::init(argc, argv, "core_node");
  ros::NodeHandle nh_;
  Continuum robot(8,4);
  bool position = false;
  Listener lt;
  pt.x = 12;
  pt.y = 12;
  pt.z = 12;
  robot.setBasePose();
  robot.update();

  ros::Subscriber poseSub  = nh_.subscribe("/poseEE",10,&Listener::pose_callback, &lt);

  if(position)
  {
      while(ros::ok())
      {
        robot.setrbShape(0,pt);
        robot.update();
        ros::spinOnce();
      }
  }
  else
  {
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