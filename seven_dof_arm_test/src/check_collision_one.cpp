#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

int main(int argc,char **argv){
  ros::init(argc,argv,"check_collision_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request,collision_result);
  ROS_INFO_STREAM("Test 1 : Initial State : " << (collision_result.collision ? "in" : "not in") << " self collision !");

  robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request,collision_result);
  ROS_INFO_STREAM("Test 2 : After setting random position : " << (collision_result.collision ? "in" : "not in") << " self collision !");

  collision_request.group_name = "arm";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request,collision_result);
  ROS_INFO_STREAM("Test 3 : After setting specified group name : " << (collision_result.collision ? "in" : "not in") << " self collision !");

  std::vector<double> joint_values;
  const robot_state::JointModelGroup* joint_model_group = current_state.getJointModelGroup("arm");
  current_state.copyJointGroupPositions(joint_model_group,joint_values);
  joint_values[0] = 1.57;
  current_state.setJointGroupPositions(joint_model_group,joint_values);
  ROS_INFO_STREAM("4. Collision points " << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

  collision_request.contacts = true;
  collision_request.max_contacts = 1000;
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request,collision_result);
  ROS_INFO_STREAM("Test 5 : After setting Contacts : " << (collision_result.collision ? "in" : "not in") << " self collision !");

  collision_detection::CollisionResult::ContactMap::const_iterator itr1;
  for(itr1 = collision_result.contacts.begin() ; itr1 != collision_result.contacts.end() ; itr1++){
    ROS_INFO("There is contacts between %s and %s" , itr1->first.first.c_str() , itr1->first.second.c_str());
  }

  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();
  collision_detection::CollisionResult::ContactMap::const_iterator itr2;
  for (itr2 = collision_result.contacts.begin(); itr2 != collision_result.contacts.end(); itr2++){
    acm.setEntry(itr2->first.first , itr2->first.second , true);
  }
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request,collision_result,copied_state,acm);
  ROS_INFO_STREAM("Test 6 : After setting ACM  : " << (collision_result.collision ? "in" : "not in") << " self collision !");

  collision_result.clear();
  planning_scene.checkCollision(collision_request,collision_result,copied_state,acm);
  ROS_INFO_STREAM("Test 7 : After setting ACM for full body :" << (collision_result.collision ? "in" : "not in") << " collision !");

  ros::shutdown();

}
