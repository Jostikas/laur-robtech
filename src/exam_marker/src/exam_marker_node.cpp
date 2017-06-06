#include <random>
#include <cmath>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "mouse_driver/mouse_event.h"

std::default_random_engine generator;
std::uniform_real_distribution<double> distribution(0.0, 1.0);

double get_rand_double()
{
	return distribution(generator);
}

std::vector<double> get_rand_quaternion()
{
	//Populate with random values
    std::vector<double> quad;
	quad.push_back(get_rand_double());
	quad.push_back(get_rand_double());
	quad.push_back(get_rand_double());
	quad.push_back(get_rand_double());
	double norm = 0;
	//normalize
	for (double c : quad)
	{
		norm += pow(c, 2);
	}
	norm = sqrt(norm);
	for (auto it = quad.begin() ; it != quad.end() ; it++)
	{
	    *it = *it/norm;
	}
	return quad;
}

    visualization_msgs::Marker get_random_marker()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "marker_frame";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
	auto quad = get_rand_quaternion();
	marker.pose.orientation.x = quad[0];
	marker.pose.orientation.y = quad[1];
	marker.pose.orientation.z = quad[2];
	marker.pose.orientation.w = quad[3];
	marker.scale.x = 3 * get_rand_double();
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = get_rand_double();
	marker.color.g = get_rand_double();
	marker.color.b = get_rand_double();
	return marker;
}

class MouseToMarker{

  public:
    MouseToMarker()
    {
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
	mouse_sub = n.subscribe("events", 2, &SubscribeAndPublish::callback, this);
    }

    void mouse_callback(const mouse_driver::mouse_event& evt)
	{
	    visualization_msgs::Marker marker = get_random_marker();

	    marker_pub.publish(marker);
	    ROS_INFO("Published marker.");
	}
  private:
  	ros::NodeHandle n;
  	ros::Publisher marker_pub;
	ros::Subscriber mouse_sub;
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "exam_marker_node");
	auto worker = MouseToMarker();
	ros::Rate loop_rate(100);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}