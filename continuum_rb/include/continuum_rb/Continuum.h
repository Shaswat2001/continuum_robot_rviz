
#ifndef CONTINUUM_RB_INCLUDE_CONTINUUM_H
#define CONTINUUM_RB_INCLUDE_CONTINUUM_H
#include "ros/ros.h"
#include <ros/package.h>
#include "tf/transform_broadcaster.h"
#include <fstream>
#include <vector>
#include "visualization_msgs/MarkerArray.h"
#include <termios.h>
#include <math.h>

using namespace std;
#define RESOLUTION 200
#define delay 1

class Continuum
{
    private:
        tf::Transform* rbTFframe;
        tf::Transform BasePose;
        tf::Transform EEPose;
        tf::TransformBroadcaster rbBroadcast;
        ros::Publisher cableTopic;
        visualization_msgs::MarkerArray cableMarker;
        double kappa;
        geometry_msgs::Point position;
        double phi;
        ofstream robotURDFfile;
        tf::Quaternion getDiskRotation(int diskID);
        void initCableMarkers();
        void creatURDF(int ndisks,double segLength,double radius);
    public:

        Continuum(int noDisks,double rbLength);
        int no_disks;
        double rb_length;
        void setBasePose();
        vector<double> arrange(double initial, double final);
        void setrbShape(double phi);
        void pose_callback(const geometry_msgs::Point msg);
        void update();

        virtual ~Continuum();
};

#endif
