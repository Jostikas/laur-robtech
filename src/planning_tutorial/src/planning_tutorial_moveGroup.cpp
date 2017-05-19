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
	ros::Rate loop_rate=100;

	moveit::planning_interface::MoveGroup group("right_arm");
	moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

	ROS_INFO("Ref frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("Effector: %s", group.getEndEffectorLink().c_str());

	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.0;
	target_pose1.position.y = -0.7;
	target_pose1.position.z = 1.0;
	group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	ROS_INFO("HERE!");
	group.move();

	//ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	//sleep(5.0);

    ROS_INFO("Visualizing plan 1 (again)");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    //sleep(5.0);
	ROS_INFO("Snooze!");

	return 0;
}