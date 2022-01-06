#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc,char **argv){
  ros::init(argc,argv,"add_collision_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planning_scene;
  sleep(3);

  moveit_msgs::CollisionObject collision_object;
  collision_object.id = "seven_dof_arm_cylinder";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.5;
  primitive.dimensions[1] = 0.2;
  primitive.dimensions[2] = 0.2;

  geometry_msgs::Pose primitive_pose;
  primitive_pose.orientation.w = 1.0;
  primitive_pose.position.x =  0.0;
  primitive_pose.position.y = -0.4;
  primitive_pose.position.z =  0.4;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(primitive_pose);
  collision_object.operation = collision_object.ADD;
  collision_object.header.frame_id = "base_link";

  std::vector<moveit_msgs::CollisionObject> collision;
  collision.push_back(collision_object);

  planning_scene.addCollisionObjects(collision);
  sleep(2.0);
  ros::waitForShutdown();
}
