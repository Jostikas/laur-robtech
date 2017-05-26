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

	moveit::planning_interface::MoveGroup group("sia5d");
	moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

	ROS_INFO("Ref frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("Effector: %s", group.getEndEffectorLink().c_str());
	auto namedTargets = group.getNamedTargets();
	for (std::string target : group.getNamedTargets())
	{
		ROS_INFO("Target: %s", target.c_str());
	}
	group.setNamedTarget("touch_ground");

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.move();

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(5.0);

	geometry_msgs::Pose target_pose1 = group.getCurrentPose().pose;
	target_pose1.orientation.x = 1;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	target_pose1.position.x -= 0.3;
	target_pose1.position.z = 0;
	group.setPoseTarget(target_pose1);
	success = group.move();

	ROS_INFO("Pointing to the ground %s", success ? "succeeded" : "FAILED");
	sleep(1);

	geometry_msgs::Pose target_pose2 = target_pose1;
	target_pose2.position.x += 0.2;
	target_pose2.position.y += 0.2;

	geometry_msgs::Pose target_pose3 = target_pose1;
	target_pose3.position.x += 0.2;
	target_pose3.position.y -= 0.2;

	std::vector<geometry_msgs::Pose> triangle;
	triangle.push_back(target_pose2);
	triangle.push_back(target_pose3);
	triangle.push_back(target_pose1);

	moveit_msgs::RobotTrajectory trajectory;
	double frac = group.computeCartesianPath(triangle,
								0.01,
								0.0,
								trajectory);
	ROS_INFO("Planned cartesian path, %f", frac);
	sleep(5);

	/* Example for actually working with the trajectory returned from computeCartesianPath:
	https://groups.google.com/d/msg/moveit-users/x5FwalM5ruk/WIj2ruYs4RwJ
	*/
	/*
	robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "sia5d");
	rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
	rt.getRobotTrajectoryMsg(trajectory);
	*/
	my_plan.trajectory_ = trajectory;

	ROS_INFO("Excecuting");
	group.execute(my_plan);

	ROS_INFO("Snooze!");

	return 0;
}