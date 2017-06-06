#include <random>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

std::default_random_engine generator;
std::uniform_real_distribution<double> distribution(0.0, 1.0);

double get_rand_double()
{
	return distribution(generator);
}

visualization_msgs::Marker get_random_marker() 
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = get_rand_double();
    marker.color.g = get_rand_double();
    marker.color.b = get_rand_double();
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    return marker;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "exam_marker_node");

	ros::NodeHandle n;

	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

	ros::Rate loop_rate(1);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique string for each message.
	 */
	int count = 0;
	while (ros::ok())
	{
		visualization_msgs::Marker marker = get_random_marker();

		marker_pub.publish(marker);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}