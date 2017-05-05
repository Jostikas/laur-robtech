#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>







int main(int argc, char **argv) {
	ros::init(argc, argv, "moveGroup");

	ros::NodeHandle node_handle;
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;
	ros::Rate loop_rate=100;

	moveit::planning_interface::MoveGroup group("right_arm");
	moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

	ROS_INFO("Ref frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("Effector: %s", group.getEndEffectorLink().c_str());
	return 0;
}