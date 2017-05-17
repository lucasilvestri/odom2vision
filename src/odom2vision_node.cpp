#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>

class oneNode
{
public:
	ros::NodeHandle node;
	ros::Subscriber sub;
	ros::Publisher pub;
	tf::TransformListener listener;
	tf::StampedTransform transform;

public:

	void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void init();

};

void oneNode::init ()
{
	this->sub = this->node.subscribe("/odometry/filtered", 1000, &oneNode::chatterCallback, this);
	this->pub = this->node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov", 1000);
}

void oneNode::chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	try
	{
        //listener.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(1.0));
		listener.lookupTransform("/map", "/base_link", ros::Time(0), this->transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_INFO("%s",ex.what());
		return;
	}

	geometry_msgs::PoseWithCovarianceStamped new_msg;
	new_msg.header = msg->header;
	new_msg.header.frame_id = "map";
	new_msg.pose =  msg->pose;

	// modify the msg
	geometry_msgs::Pose new_pose;
	tf::poseTFToMsg(this->transform, new_pose);
	new_msg.pose.pose = new_pose;

	//std::cout << __func__ << new_pose.position << std::endl;

	this->pub.publish(new_msg);
	ros::spinOnce();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "read");
	oneNode mynode;
	mynode.init();

	ros::spin();
	return 0;
}


