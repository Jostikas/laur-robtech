#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

const bool revizualise = true;

int main(int argc, char **argv) {
	ros::init(argc, argv, "moveGroup");

	ros::NodeHandle node_handle;
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup left_g("left_manip");
	moveit::planning_interface::MoveGroup right_g("right_manip");
	moveit::planning_interface::MoveGroup both_g("both_manip");
	moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

	ROS_INFO("Ref frame left: %s", left_g.getPlanningFrame().c_str());
	ROS_INFO("Effector left: %s", left_g.getEndEffectorLink().c_str());
	ROS_INFO("Ref frame right: %s", right_g.getPlanningFrame().c_str());
	ROS_INFO("Effector right: %s", right_g.getEndEffectorLink().c_str());
	ROS_INFO("Ref frame both: %s", both_g.getPlanningFrame().c_str());
	ROS_INFO("Effector both: %s", both_g.getEndEffectorLink().c_str());


	ROS_INFO("Random joint values for left manipulator");
	left_g.setRandomTarget();
	left_g.move();  //Blocking call...
	sleep(5);

	ROS_INFO("Random joint values for right manipulator");
	right_g.setRandomTarget();
	right_g.move();
	sleep(5);

	ROS_INFO("Random joint values for both manipulator");
	both_g.setRandomTarget();
	both_g.move();

	sleep(5);

	ROS_INFO("Aping Michelangelo");
	both_g.setNamedTarget("twiddle");
	both_g.move();
	sleep(5);

	ROS_INFO("Snooze!");

	return 0;
}