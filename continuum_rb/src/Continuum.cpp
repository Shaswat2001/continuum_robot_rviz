#include <continuum_rb/Continuum.h>
#include <iostream>

Continuum::Continuum(int noDisks,double rbLength)
{
    ros::NodeHandle nh;
    this->no_disks = noDisks;
    this->rb_length = rbLength;
	// Setting the initial position of the robot
	this->position.x = 12;
	this->position.y = 12;
	this->position.z = 12;
	this->kappa = 2/rb_length*(atan(-double(this->position.x)/double(this->position.z)));
    this->rbTFframe = new tf::Transform[noDisks];
    cableMarker.markers.resize(RESOLUTION);
    cableTopic = nh.advertise<visualization_msgs::MarkerArray>("cable_topic",1);
    this->phi = double(noDisks);
    this->creatURDF(noDisks,rbLength,0.3);
}
void Continuum::setBasePose()
{

    BasePose.setOrigin(tf::Vector3(0,0,8));
    BasePose.setRotation(tf::Quaternion(0,0,0,1));
    EEPose.setOrigin(tf::Vector3(0,0,8-rb_length));
    EEPose.setRotation(tf::Quaternion(0,0,0,1));
    
    kappa = 0.001;
    phi = 0;

    initCableMarkers();
}

void Continuum::initCableMarkers()
{
    uint32_t shape = visualization_msgs::Marker::SPHERE;

	  for(int r=0; r<RESOLUTION; r++)
	  {

	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    cableMarker.markers[r].header.frame_id = "base_link";
	    cableMarker.markers[r].header.stamp = ros::Time::now();

	    // Set the namespace and id for this marker.  This serves to create a unique ID
	    // Any marker sent with the same namespace and id will overwrite the old one
	    cableMarker.markers[r].ns = "basic_shapes";
	    cableMarker.markers[r].id = r;

	    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	    cableMarker.markers[r].type = shape;

	    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	    cableMarker.markers[r].action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    cableMarker.markers[r].scale.x = .1;
	    cableMarker.markers[r].scale.y = .1;
	    cableMarker.markers[r].scale.z = .1;

	    // Set the color -- be sure to set alpha to something non-zero!
	    cableMarker.markers[r].color.b = 1.0f;
	    cableMarker.markers[r].color.a = 1.0;
	    cableMarker.markers[r].lifetime = ros::Duration();
	  }
}
void Continuum::setrbShape(double phi)
{

	double kappa_initial = this->kappa;
	double kappa_new = 2/rb_length*(atan(-double(this->position.x)/double(this->position.z)));
	vector<double> values = this->arrange(kappa_initial,kappa_new);
	for(int i=0;i<values.size();i++)
	{
		this->kappa = values[i];
		tf::Matrix3x3 R;
		tf::Quaternion qRot;

		R.setValue(pow(cos(phi),2) * (cos(kappa*rb_length) - 1) + 1, sin(phi)*cos(phi)*( cos(kappa*rb_length) - 1), -cos(phi)*sin(kappa*rb_length),
							sin(phi)*cos(phi)*( cos(kappa*rb_length) - 1), pow(cos(phi),2) * ( 1 - cos(kappa*rb_length) ) + cos( kappa * rb_length ),  -sin(phi)*sin(kappa*rb_length),
							cos(phi)*sin(kappa*rb_length),  sin(phi)*sin(kappa*rb_length), cos(kappa*rb_length));
		R.getRotation(qRot);

		EEPose.setRotation(BasePose.getRotation() * qRot);

		tf::Vector3 eePosition = BasePose.getOrigin() + ( tf::Matrix3x3(BasePose.getRotation())*tf::Vector3(cos(phi)*( cos(kappa*rb_length) - 1)/kappa, sin(phi)*( cos(kappa*rb_length) - 1)/kappa, sin(kappa*rb_length)/kappa));
		EEPose.setOrigin(eePosition);
		this->update();
		ros::spinOnce();
	}
}
tf::Quaternion Continuum::getDiskRotation(int diskID)
{
    tf::Matrix3x3 Rot;
    tf::Quaternion qRot;
    Rot.setValue(pow(cos(phi),2) * (cos(kappa*((diskID/((double)no_disks))*rb_length)) - 1) + 1, sin(phi)*cos(phi)*( cos(kappa*((diskID/((double)no_disks))*rb_length)) - 1), -cos(phi)*sin(kappa*((diskID/((double)no_disks))*rb_length)),
                            sin(phi)*cos(phi)*( cos(kappa*((diskID/((double)no_disks))*rb_length)) - 1), pow(cos(phi),2) * ( 1 - cos(kappa*((diskID/((double)no_disks))*rb_length)) ) + cos( kappa * ((diskID/((double)no_disks))*rb_length)),  -sin(phi)*sin(kappa*((diskID/((double)no_disks))*rb_length)),
                            cos(phi)*sin(kappa*((diskID/((double)no_disks))*rb_length)),  sin(phi)*sin(kappa*((diskID/((double)no_disks))*rb_length)), cos(kappa*((diskID/((double)no_disks))*rb_length)));
    Rot.getRotation(qRot);
    //endEffectorPose[segID].setRotation(BasePose.getRotation() * qRot);
    return qRot;
}
void Continuum::update(void)
{
    ros::Rate rate(15);
    tf::Vector3 eeP;
    tf::Vector3 eePc;
    char link_name[30];

    for(int i = 0;i<=no_disks;i++)
    {
        eeP[0]=cos(phi)*(cos(kappa*((i/((double)no_disks))*rb_length)) - 1)/kappa;
        eeP[1]=sin(phi)*(cos(kappa*((i/((double)no_disks))*rb_length)) - 1)/kappa;
        eeP[2]=(sin(kappa*((i/((double)no_disks))*rb_length))/kappa);;
        eeP = tf::Matrix3x3(BasePose.getRotation())*eeP;

        this->rbTFframe[i].setOrigin(tf::Vector3(BasePose.getOrigin().x() - eeP.getX(), BasePose.getOrigin().y() - eeP.getY(), BasePose.getOrigin().z() - eeP.getZ()) );
	    this->rbTFframe[i].setRotation(BasePose.getRotation() * getDiskRotation(i));

        sprintf(link_name,"link%d",i);
        rbBroadcast.sendTransform(tf::StampedTransform(rbTFframe[i],ros::Time::now(),"base_link",link_name));
    }
    for(int i=0;i<RESOLUTION&&ros::ok();i++)
    {
		eePc[0] = cos(phi)*(cos(kappa*((i/((double)RESOLUTION-1))*rb_length)) - 1)/kappa;
		eePc[1] =  sin(phi)*(cos(kappa*((i/((double)RESOLUTION-1))*rb_length)) - 1)/kappa;
		eePc[2] = (sin(kappa*((i/((double)RESOLUTION-1))*rb_length))/kappa);

		eePc =  tf::Matrix3x3(BasePose.getRotation())*eePc;
		cableMarker.markers[i].pose.position.x = BasePose.getOrigin().x()- eePc[0];
				cableMarker.markers[i].pose.position.y = BasePose.getOrigin().y()- eePc[1];
				cableMarker.markers[i].pose.position.z = BasePose.getOrigin().z()- eePc[2];
		// Slerp for spherical interpolation
		//	slerpQuaternionCable = BasePose.getRotation().slerp(endEffectorPose[segID].getRotation(),(double)((i/((double)RESOLUTION-1))));
			cableMarker.markers[i].pose.orientation.x = 0;//slerpQuaternionCable.x();
			cableMarker.markers[i].pose.orientation.y = 0;//slerpQuaternionCable.y();
			cableMarker.markers[i].pose.orientation.z = 0;//slerpQuaternionCable.z();
			cableMarker.markers[i].pose.orientation.w = 1;//slerpQuaternionCable.w();

	}
	cableTopic.publish(cableMarker);

	rate.sleep();
}

void Continuum::pose_callback(const geometry_msgs::Point msg)
{
    this->position = msg;
}

void Continuum::creatURDF(int ndisks,double segLength,double radius)
{
    std::string path= ros::package::getPath("continuum_rb");  path =path+ "/urdf/rb_model.urdf";
    remove(path.c_str());

    robotURDFfile.open (path.c_str(), std::fstream::app);
    robotURDFfile << "<?xml version=\"1.0\"?>" <<endl;
    robotURDFfile << "<robot name=\"continuum_robot\">"<<endl;
    robotURDFfile << "<link name=\"base_link\">"<<endl;
	robotURDFfile <<"</link>"<<endl;
    robotURDFfile << "<material name=\"white\">"<<endl;
    robotURDFfile << "<color rgba=\"0 1 0 1\"/>"<<endl;
    robotURDFfile << "</material>"<<endl;
    robotURDFfile<<endl;

    for(int disk=0;disk<=ndisks;disk++)
	  {
		  robotURDFfile << "<link name=\"link"<<disk<<"\">"<<endl;
		  robotURDFfile << "<visual>"<<endl;
		  robotURDFfile << "<geometry>"<<endl;
		  robotURDFfile << "<cylinder length=\"0.1\" radius=\""<<radius<<"\"/>"<<endl;
		  robotURDFfile << "<origin rpy=\"0 0 0\" xyz=\"0 0 "<<8-(disk/(ndisks-1))*segLength<<"\"/>"<<endl;
		  robotURDFfile << "</geometry>"<<endl;
		  robotURDFfile <<"</visual>"<<endl;
/*		  robotURDFfile <<"<collision>"<<endl;
		  robotURDFfile <<"<geometry>"<<endl;
		  robotURDFfile << "<cylinder length=\"0.1\" radius=\""<<radius<<"\"/>"<<endl;
		  robotURDFfile << "</geometry>"<<endl;
		  robotURDFfile <<"</collision>"<<endl;*/
		  robotURDFfile <<"</link>"<<endl;
		  robotURDFfile <<endl;
		  robotURDFfile<<"<joint name=\"joint"<<disk<<"\" type=\"floating\">"<<endl;
		  robotURDFfile<<"<parent link=\"base_link\"/>"<<endl;
		  robotURDFfile<<"<child link=\"link"<<disk<<"\"/>"<<endl;
		  robotURDFfile<<"</joint>"<<endl;
		  robotURDFfile<<endl;

	  }
      robotURDFfile <<"</robot>"<<endl;
      robotURDFfile.close();
}

vector<double> Continuum::arrange(double initial, double final)
{
	vector<double> values_kappa;
	if(initial < final)
	{
		for(double i=initial;i<final;i=i+0.01)
			values_kappa.push_back(i);
	}
	else
	{
		for(double i=initial;i<final;i=i-0.01)
			values_kappa.push_back(i);
	}

	return values_kappa;
}
Continuum::~Continuum() {
	// TODO Auto-generated destructor stub

}