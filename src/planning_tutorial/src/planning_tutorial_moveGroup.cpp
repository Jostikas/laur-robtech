#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>

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
	//group.setOrientationTarget(0, 0, 1, 0);
	//geometry_msgs::Pose target_pose1 = group.getPoseTarget().pose;
	//group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	ROS_INFO("HERE!");
	bool success = group.move();

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(5.0);

	geometry_msgs::Pose target_pose1 = group.getCurrentPose().pose;
	target_pose1.orientation.x = 1;
	target_pose1.orientation.y = 0;
	target_pose1.orientation.z = 0;
	group.setPoseTarget(target_pose1);
	success = group.move();

	ROS_INFO("Pointing %s", success ? "succeeded" : "FAILED");
	sleep(2.0);

	geometry_msgs::Pose target_pose2 = target_pose1;
	target_pose2.position.x;
	target_pose2.position.y += 0.1;

	geometry_msgs::Pose target_pose3 = target_pose1;
	target_pose2.position.x += 0.1;
	target_pose2.position.y -= 0.1;

	std::vector<geometry_msgs::Pose> triangle;
	triangle.push_back(target_pose1);
	triangle.push_back(target_pose2);
	triangle.push_back(target_pose3);
	int corner = 1;

	group.setPoseTarget(triangle[1]);
	group.move();
	sleep(5);
	group.setPoseTarget(triangle[2]);
	group.move();
	sleep(5);
	group.setPoseTarget(triangle[0]);
	group.move();
	sleep(5);
	group.setPoseTarget(triangle[1]);
	group.move();
	sleep(5);
	group.setPoseTarget(triangle[2]);
	group.move();
	sleep(5);
	group.setPoseTarget(triangle[0]);
	group.move();
	sleep(5);

	// while (true)
	// {
	//     ROS_INFO("Corner %d", corner);
	//     group.setPoseTarget(triangle[corner++]);
	//     group.move();
	//     corner = corner % 3;
	// 	sleep(5);
	// }

	ROS_INFO("Visualizing plan 1 (again)");
	display_trajectory.trajectory_start = my_plan.start_state_;
	display_trajectory.trajectory.push_back(my_plan.trajectory_);
	display_publisher.publish(display_trajectory);
	/* Sleep to give Rviz time to visualize the plan. */
	//sleep(5.0);
	ROS_INFO("Snooze!");

	return 0;
}