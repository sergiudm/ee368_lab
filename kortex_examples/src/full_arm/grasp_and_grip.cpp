/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2019 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */
#include <atomic>
#include <thread>

#include "ros/console.h"
#include "ros/ros.h"

#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/ExecuteWaypointTrajectory.h>
#include <kortex_driver/GetProductConfiguration.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/ModelId.h>
#include <kortex_driver/OnNotificationActionTopic.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/ValidateWaypointList.h>
#include <vector>

#define HOME_ACTION_IDENTIFIER 2

std::atomic<int> last_action_notification_event{0};

void notification_callback(const kortex_driver::ActionNotification &notif) {
  last_action_notification_event = notif.action_event;
}

bool wait_for_action_end_or_abort() {
  while (ros::ok()) {
    if (last_action_notification_event.load() ==
        kortex_driver::ActionEvent::ACTION_END) {
      ROS_INFO("Received ACTION_END notification");
      return true;
    } else if (last_action_notification_event.load() ==
               kortex_driver::ActionEvent::ACTION_ABORT) {
      ROS_INFO("Received ACTION_ABORT notification");
      return false;
    }
    ros::spinOnce();
  }
  return false;
}

kortex_driver::Waypoint FillCartesianWaypoint(float new_x, float new_y,
                                              float new_z, float new_theta_x,
                                              float new_theta_y,
                                              float new_theta_z,
                                              float blending_radius) {
  kortex_driver::Waypoint waypoint;
  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = new_x;
  cartesianWaypoint.pose.y = new_y;
  cartesianWaypoint.pose.z = new_z;
  cartesianWaypoint.pose.theta_x = new_theta_x;
  cartesianWaypoint.pose.theta_y = new_theta_y;
  cartesianWaypoint.pose.theta_z = new_theta_z;
  cartesianWaypoint.reference_frame =
      kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = blending_radius;

  waypoint.oneof_type_of_waypoint.cartesian_waypoint.push_back(
      cartesianWaypoint);

  return waypoint;
}

bool example_clear_faults(ros::NodeHandle n, const std::string &robot_name) {
  ros::ServiceClient service_client_clear_faults =
      n.serviceClient<kortex_driver::Base_ClearFaults>("/" + robot_name +
                                                       "/base/clear_faults");
  kortex_driver::Base_ClearFaults service_clear_faults;

  // Clear the faults
  if (!service_client_clear_faults.call(service_clear_faults)) {
    std::string error_string = "Failed to clear the faults";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

bool example_home_the_robot(ros::NodeHandle n, const std::string &robot_name) {
  ros::ServiceClient service_client_read_action =
      n.serviceClient<kortex_driver::ReadAction>("/" + robot_name +
                                                 "/base/read_action");
  kortex_driver::ReadAction service_read_action;
  last_action_notification_event = 0;

  // The Home Action is used to home the robot. It cannot be deleted and is
  // always ID #2:
  service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

  if (!service_client_read_action.call(service_read_action)) {
    std::string error_string = "Failed to call ReadAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // We can now execute the Action that we read
  ros::ServiceClient service_client_execute_action =
      n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name +
                                                    "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input = service_read_action.response.output;

  if (service_client_execute_action.call(service_execute_action)) {
    ROS_INFO("The Home position action was sent to the robot.");
  } else {
    std::string error_string = "Failed to call ExecuteAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}

bool example_set_cartesian_reference_frame(ros::NodeHandle n,
                                           const std::string &robot_name) {
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame =
      n.serviceClient<kortex_driver::SetCartesianReferenceFrame>(
          "/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame
      service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame =
      kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_MIXED;
  if (!service_client_set_cartesian_reference_frame.call(
          service_set_cartesian_reference_frame)) {
    std::string error_string = "Failed to call SetCartesianReferenceFrame";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  return true;
}

bool example_send_gripper_command(ros::NodeHandle n,
                                  const std::string &robot_name, double value) {
  // Initialize the ServiceClient
  ros::ServiceClient service_client_send_gripper_command =
      n.serviceClient<kortex_driver::SendGripperCommand>(
          "/" + robot_name + "/base/send_gripper_command");
  kortex_driver::SendGripperCommand service_send_gripper_command;

  // Initialize the request
  kortex_driver::Finger finger;
  finger.finger_identifier = 0;
  finger.value = value;
  service_send_gripper_command.request.input.gripper.finger.push_back(finger);
  service_send_gripper_command.request.input.mode =
      kortex_driver::GripperMode::GRIPPER_POSITION;

  if (service_client_send_gripper_command.call(service_send_gripper_command)) {
    ROS_INFO("The gripper command was sent to the robot.");
  } else {
    std::string error_string = "Failed to call SendGripperCommand";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  return true;
}

bool move_to_pose(ros::NodeHandle n, const std::string &robot_name,
                  const std::vector<float> &goal_pose) {

  ros::ServiceClient service_client_execute_waypoints_trajectory =
      n.serviceClient<kortex_driver::ExecuteWaypointTrajectory>(
          "/" + robot_name + "/base/execute_waypoint_trajectory");
  ros::ServiceClient service_client_get_config =
      n.serviceClient<kortex_driver::GetProductConfiguration>(
          "/" + robot_name + "/base/get_product_configuration");

  kortex_driver::ExecuteWaypointTrajectory service_execute_waypoints_trajectory;
  kortex_driver::GetProductConfiguration service_get_config;

  last_action_notification_event = 0;

  if (!service_client_get_config.call(service_get_config)) {
    std::string error_string = "Failed to call GetProductConfiguration";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  auto product_config = service_get_config.response.output;

  auto goal_x = goal_pose[0];
  auto goal_y = goal_pose[1];
  auto goal_z = goal_pose[2];
  auto goal_theta_x = goal_pose[3];
  auto goal_theta_y = goal_pose[4];
  auto goal_theta_z = goal_pose[5];

  if (product_config.model ==
      kortex_driver::ModelId::MODEL_ID_L31) // If the robot is a GEN3-LITE use
                                            // this trajectory.
  {
    service_execute_waypoints_trajectory.request.input.waypoints.push_back(
        FillCartesianWaypoint(goal_x, goal_y, goal_z, goal_theta_x,
                              goal_theta_y, goal_theta_z, 0));
  } else {
    ROS_INFO("only support gen3-lite");
    return false;
  }
  service_execute_waypoints_trajectory.request.input.duration = 0;
  service_execute_waypoints_trajectory.request.input.use_optimal_blending = 0;
  if (service_client_execute_waypoints_trajectory.call(
          service_execute_waypoints_trajectory)) {
    ROS_INFO("The Waypoint2 command was sent to the robot.");
  } else {
    std::string error_string = "Failed to call ExecuteWaypointTrajectory";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grasp_and_grip_cpp");

  // For testing purpose
  // ros::param::del("/kortex_examples_test_results/full_arm_movement_cpp");

  bool success = true;

  //*******************************************************************************
  // ROS Parameters
  ros::NodeHandle n;

  std::string robot_name = "my_gen3_lite";

  int degrees_of_freedom = 6;

  bool is_gripper_present = true;

  // Parameter robot_name
  if (!ros::param::get("~robot_name", robot_name)) {
    std::string error_string =
        "Parameter robot_name was not specified, defaulting to " + robot_name +
        " as namespace";
    ROS_WARN("%s", error_string.c_str());
  } else {
    std::string error_string =
        "Using robot_name " + robot_name + " as namespace";
    ROS_INFO("%s", error_string.c_str());
  }

  // Parameter degrees_of_freedom
  if (!ros::param::get("/" + robot_name + "/degrees_of_freedom",
                       degrees_of_freedom)) {
    std::string error_string =
        "Parameter /" + robot_name +
        "/degrees_of_freedom was not specified, defaulting to " +
        std::to_string(degrees_of_freedom) + " as degrees of freedom";
    ROS_WARN("%s", error_string.c_str());
  } else {
    std::string error_string = "Using degrees_of_freedom " +
                               std::to_string(degrees_of_freedom) +
                               " as degrees_of_freedom";
    ROS_INFO("%s", error_string.c_str());
  }

  // Parameter is_gripper_present
  if (!ros::param::get("/" + robot_name + "/is_gripper_present",
                       is_gripper_present)) {
    std::string error_string =
        "Parameter /" + robot_name +
        "/is_gripper_present was not specified, defaulting to " +
        std::to_string(is_gripper_present);
    ROS_WARN("%s", error_string.c_str());
  } else {
    std::string error_string =
        "Using is_gripper_present " + std::to_string(is_gripper_present);
    ROS_INFO("%s", error_string.c_str());
  }

  std::vector<float> pose1;
  std::vector<float> pose2;
  if (!ros::param::get("/coords/src_pose", pose1)) {
    ROS_ERROR("Failed to get src_pose");
    return -1;
  }
  if (!ros::param::get("/coords/goal_pose", pose2)) {
    ROS_ERROR("Failed to get goal_pose");
    return -1;
  }
  //*******************************************************************************

  // Subscribe to the Action Topic
  ros::Subscriber sub = n.subscribe("/" + robot_name + "/action_topic", 1000,
                                    notification_callback);

  // We need to call this service to activate the Action Notification on the
  // kortex_driver node.
  ros::ServiceClient service_client_activate_notif =
      n.serviceClient<kortex_driver::OnNotificationActionTopic>(
          "/" + robot_name + "/base/activate_publishing_of_action_topic");
  kortex_driver::OnNotificationActionTopic service_activate_notif;
  if (service_client_activate_notif.call(service_activate_notif)) {
    ROS_INFO("Action notification activated!");
  } else {
    std::string error_string = "Action notification publication failed";
    ROS_ERROR("%s", error_string.c_str());
    success = false;
  }

  //*******************************************************************************
  // Make sure to clear the robot's faults else it won't move if it's already in
  // fault
  success &= example_clear_faults(n, robot_name);

  //*******************************************************************************
  // Move the robot to the Home position with an Action
  success &= example_home_the_robot(n, robot_name);

  //*******************************************************************************
  // Set the reference frame to "Mixed"
  success &= example_set_cartesian_reference_frame(n, robot_name);

  success &= example_send_gripper_command(n, robot_name, 0.1);
  ROS_INFO("open the gripper");

  //*******************************************************************************
  success &= move_to_pose(n, robot_name, pose1);

  success &= example_send_gripper_command(n, robot_name, 0.8);
  ROS_INFO("close the gripper");

  success &= move_to_pose(n, robot_name, pose2);

  success &= example_send_gripper_command(n, robot_name, 0.1);
  ROS_INFO("open the gripper");

  //*******************************************************************************
  // Move the robot to the Home position one last time.
  success &= example_home_the_robot(n, robot_name);
  //*******************************************************************************
  ROS_INFO("Grasp and grip finished with %s", success ? "success" : "failure");

  return success ? 0 : 1;
}