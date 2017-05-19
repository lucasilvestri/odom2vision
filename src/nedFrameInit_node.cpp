#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_listener.h>

class oneNode
{
public:
	ros::NodeHandle node;
	ros::Subscriber sub;
	bool initialized;
	geometry_msgs::Quaternion q;

public:

	void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void init();

};

void oneNode::init ()
{
	this->sub = this->node.subscribe("/mavros/imu/data", 1000, &oneNode::chatterCallback, this);
}

void oneNode::chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	if(initialized)
	{
		//read orientation
		this->q = msg->orientation;
		this->initialized = false;
	}

	ros::spinOnce();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "read");
	oneNode mynode;
	mynode.initialized = true;
	mynode.init();
	ros::spin();

	while(true)
	{
		//publish world-map

	}
	return 0;
}

