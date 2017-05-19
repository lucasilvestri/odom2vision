#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>

class oneNode
{
public:
	ros::NodeHandle node;
	ros::Subscriber sub;
	ros::Subscriber subImu;
	ros::Publisher pub;
	tf::TransformListener listener;
	tf::StampedTransform transform;	
	bool initialized;
	geometry_msgs::Quaternion quat;

public:

	void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void init();

};

void oneNode::init ()
{
	this->sub = this->node.subscribe("/odometry/filtered", 1000, &oneNode::chatterCallback, this);
	this->sub = this->node.subscribe("/mavros/imu/data", 1000, &oneNode::imuCallback, this);
	this->pub = this->node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov", 1000);
	this->initialized = true;
}

void oneNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	if(initialized)
	{
		//read orientation
		this->quat = msg->orientation;
		this->initialized = false;
	}

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	tf::Quaternion q; 
	//q.setRPY(0, 0, 0);
	tf::quaternionMsgToTF(this->quat, q);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world2", "map2")); 
	ROS_INFO("tf");
}

void oneNode::chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	try
	{
                //listener.waitForTransform("/map", "/base_link", ros::Time::now(), ros::Duration(1.0));
                //listener.waitForTransform("/map", "/base_link", msg->header.stamp, ros::Duration(2.0));
		listener.waitForTransform("/world", "/base_link", ros::Time(0), ros::Duration(2.0));

		listener.lookupTransform("/world", "/base_link", ros::Time(0), this->transform);
                //listener.lookupTransform("/map", "/base_link", ros::Time::now(), this->transform);
                //listener.lookupTransform("/map", "/base_link", msg->header.stamp, this->transform);

	}
	catch (tf::TransformException ex)
	{
		ROS_INFO("%s",ex.what());
		return;
	}

	geometry_msgs::PoseWithCovarianceStamped new_msg;
	new_msg.header = msg->header;
	new_msg.header.frame_id = "world";
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


