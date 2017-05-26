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

	moveit::planning_interface::MoveGroup group("right_arm");
	moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

	ROS_INFO("Ref frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("Effector: %s", group.getEndEffectorLink().c_str());
	auto namedTargets = group.getNamedTargets();
	ROS_INFO("Named targets:");
	for (std::string target : group.getNamedTargets())
	{
		ROS_INFO("Target: %s", target.c_str());
	}
	//group.setNamedTarget("touch_ground");
	//group.setOrientationTarget(0, 0, 1, 0);
	//geometry_msgs::Pose target_pose1 = group.getPoseTarget().pose;
	//group.setPoseTarget(target_pose1);

	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.w = 1.0;
	target_pose1.position.x = 0.28;
	target_pose1.position.y = -0.7;
	target_pose1.position.z = 1.0;
	group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"succeeded":"FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(3.0);

	ROS_INFO("Visualizing plan 1 (again)");
	display_trajectory.trajectory_start = my_plan.start_state_;
	display_trajectory.trajectory.push_back(my_plan.trajectory_);
	display_publisher.publish(display_trajectory);
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(3.0);

	group.setNamedTarget("tuck_right_arm");
	success = group.plan(my_plan);
	ROS_INFO("Visualizing named pose goal tuck_right_arm %s", success ? "succeeded" : "FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(3.0);

	std::vector<double> group_variable_values;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
	group_variable_values[0] = -1.0;
	group.setJointValueTarget(group_variable_values);
	success = group.plan(my_plan);
	ROS_INFO("Visualizing joint value target as in tutorial %s", success ? "succeeded" : "FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(3.0);

	group_variable_values = group.getCurrentJointValues();
	group_variable_values[0] = -1.0;
	group.setJointValueTarget(group_variable_values);
	success = group.plan(my_plan);
	ROS_INFO("Visualizing joint value target via getCurrentJointValues %s", success ? "succeeded" : "FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(3.0);

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "r_wrist_roll_link";
	ocm.header.frame_id = "base_link";
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	group.setPathConstraints(test_constraints);

	robot_state::RobotState start_state(*group.getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;
	start_pose2.position.x = 0.55;
	start_pose2.position.y = -0.05;
	start_pose2.position.z = 0.8;
	const robot_state::JointModelGroup *joint_model_group =
	    start_state.getJointModelGroup(group.getName());
	start_state.setFromIK(joint_model_group, start_pose2);
	group.setStartState(start_state);

	group.setPoseTarget(target_pose1);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(5.0);
	group.clearPathConstraints();

	std::vector<geometry_msgs::Pose> waypoints;

	geometry_msgs::Pose target_pose3 = start_pose2;
	target_pose3.position.x += 0.2;
	target_pose3.position.z += 0.2;
	waypoints.push_back(target_pose3); // up and out

	target_pose3.position.y -= 0.2;
	waypoints.push_back(target_pose3); // left

	target_pose3.position.z -= 0.2;
	target_pose3.position.y += 0.2;
	target_pose3.position.x -= 0.2;
	waypoints.push_back(target_pose3); // down and right (back to start)

	moveit_msgs::RobotTrajectory trajectory1;
	double fraction = group.computeCartesianPath(waypoints,
						     0.01, // eef_step
						     0.0,  // jump_threshold
						     trajectory1);
	ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
		 fraction * 100.0);
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(10.0);

	moveit_msgs::RobotTrajectory trajectory2;
	fraction = group.computeCartesianPath(waypoints,
						     0.01, // eef_step
						     1,  // jump_threshold
						     trajectory2);
	ROS_INFO("Visualizing plan 4 (cartesian path) with jump threshold (%.2f%% acheived)",
		 fraction * 100.0);
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(10.0);

/* Collision object tests */
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = group.getPlanningFrame();

	/* The id of the object is used to identify it. */
	collision_object.id = "box1";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.4;
	primitive.dimensions[1] = 0.1;
	primitive.dimensions[2] = 0.4;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = 0.6;
	box_pose.position.y = -0.4;
	box_pose.position.z = 1.2;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

	ROS_INFO("Add an object into the world");
	planningSceneInterface.addCollisionObjects(collision_objects);

	/* Sleep so we have time to see the object in RViz */
	sleep(2.0);

	group.setPlanningTime(10.0);
	group.setStartState(*group.getCurrentState());
	group.setPoseTarget(target_pose1);
	success = group.plan(my_plan);

	ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
		 success ? "" : "FAILED");
	/* Sleep to give Rviz time to visualize the plan. */
	sleep(10.0);

	ROS_INFO("Attach the object to the robot");
	group.attachObject(collision_object.id);
	/* Sleep to give Rviz time to show the object attached (different color). */
	sleep(4.0);

	success = group.plan(my_plan);
	ROS_INFO("Visualizing plan 5 (box attached) %s",
		 success ? "" : "FAILED");
	sleep(5);

	ROS_INFO("Detach the object from the robot");
	group.detachObject(collision_object.id);
	/* Sleep to give Rviz time to show the object detached. */
	sleep(4.0);

	ROS_INFO("Remove the object from the world");
	std::vector<std::string> object_ids;
	object_ids.push_back(collision_object.id);
	planningSceneInterface.removeCollisionObjects(object_ids);
	/* Sleep to give Rviz time to show the object is no longer there. */
	sleep(4.0);

	moveit::planning_interface::MoveGroup two_arms_group("arms");

	two_arms_group.setPoseTarget(target_pose1, "r_wrist_roll_link");

	geometry_msgs::Pose target_pose2;
	target_pose2.orientation.w = 1.0;
	target_pose2.position.x = 0.7;
	target_pose2.position.y = 0.15;
	target_pose2.position.z = 1.0;

	two_arms_group.setPoseTarget(target_pose2, "l_wrist_roll_link");

	moveit::planning_interface::MoveGroup::Plan two_arms_plan;
	two_arms_group.plan(two_arms_plan);
	sleep(4.0);
	return 0;
}