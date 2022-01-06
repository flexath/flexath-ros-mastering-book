#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc,char **argv){
  ros::init(argc,argv,"custom_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene;

  ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,true);
  moveit_msgs::DisplayTrajectory trajectory;

  ROS_INFO("Reference Frame : %s " , group.getPlanningFrame().c_str());
  ROS_INFO("End Effector Frame : %s" , group.getEndEffectorLink().c_str());

  geometry_msgs::Pose pose1;
  pose1.position.x = 0.0261186;
  pose1.position.y = 4.50972e-07;
  pose1.position.z = 0.573659;
  pose1.orientation.x = 4.04423e-07;
  pose1.orientation.y = -0.687396;
  pose1.orientation.z = 4.81813e-07;
  pose1.orientation.w = 0.726282;
  group.setPoseTarget(pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
  ROS_INFO("Visualizing plan 1 (pose goal) is %s",success.val ? "SUCCESSFUL":"FAILED");

  group.move();
  ros::waitForShutdown();
}
