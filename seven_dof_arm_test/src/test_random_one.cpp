#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc,char **argv){
  ros::init(argc,argv,"random_node");
  ros::AsyncSpinner spinner(5);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setRandomTarget();
  group.move();
  ros::waitForShutdown();
}
