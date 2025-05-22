#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <control_msgs/action/gripper_command.hpp>

#include <Eigen/Dense>

#include "rclcpp_action/rclcpp_action.hpp"

void gripper_cmd(rclcpp::Node::SharedPtr& node, double gripper_position)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Gripper open");
  auto action_client =
      rclcpp_action::create_client<control_msgs::action::GripperCommand>(node, "/panda_hand_controller/gripper_cmd");

  RCLCPP_INFO_STREAM(node->get_logger(), "Wait for gripper action server...");
  if (!action_client->wait_for_action_server())
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
    throw std::runtime_error("Action server not available after waiting");
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "Gripper Online!");

  auto goal_msg = control_msgs::action::GripperCommand::Goal();
  goal_msg.command.position = gripper_position;

  auto goal_handle_future = action_client->async_send_goal(goal_msg);

  if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::timeout)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Gripper motion Goal Accepted!");
    auto result_future = action_client->async_get_result(goal_handle_future.get());
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::timeout)
    {
      // result_future.get().result->reached_goal; // You can use the result if you want...
      RCLCPP_INFO_STREAM(node->get_logger(), "Gripper motion OK");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Gripper motion ERROR");
      rclcpp::shutdown();
      throw std::runtime_error("Gripper motion ERROR");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Gripper motion NOT ACCEPTED BY THE SERVER");
    rclcpp::shutdown();
    throw std::runtime_error("Gripper motion NOT ACCEPTED BY THE SERVER");
  }
}

void gripper_open(rclcpp::Node::SharedPtr& node)
{
  gripper_cmd(node, 0.04);
}

void gripper_close(rclcpp::Node::SharedPtr& node)
{
  gripper_cmd(node, 0.0);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "demo_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  // This way we don't have to spin
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  static const std::string WORLD_FRAME = "world";
  static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string EE_LINK = "panda_hand_tcp";

  moveit::planning_interface::MoveGroupInterface move_group_interface(node, PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  /* Scelgo il planner da usare */
  move_group_interface.setPlanningTime(120);
  // move_group_interface.setPlannerId("RRTstarkConfigDefault");
  move_group_interface.setPlannerId("RRTConnectkConfigDefault");
  // move_group_interface.setPlannerId("RRTConnectkConfigDefault");
  // move_group_interface.setPlannerId("PRMstarkConfigDefault");

  char ans;

  // Parametri della scena [HARD CODED!]
  double x1 = 0.5;
  double y1 = 0.60;
  double DX_0 = 0.2;
  double Dy_f = 0.25;
  // double Table_DX = 0.5;
  // double Table_DY = 1.0;
  double Table_DZ = 0.4;
  // double DX = 0.5;
  // double DX_2 = 0.25;
  // double DZ_2 = 0.5;
  // double DZ_3 = 0.25;
  double Dz_grasp = 0.2;

  gripper_open(node);
  // PLAN GRASP
  {
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

    // Eigen::Quaterniond q;
    // q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

    move_group_interface.setStartStateToCurrentState();  //<-- it is usefull to update the start state to the current
                                                         // state
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = WORLD_FRAME;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose.pose.position.x = x1 + DX_0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = Table_DZ + Dz_grasp;

    RCLCPP_INFO_STREAM(node->get_logger(), "Plan target:\n" << geometry_msgs::msg::to_yaml(pose));
    // return 0;
    move_group_interface.clearPathConstraints();
    move_group_interface.setStartStateToCurrentState();
    move_group_interface.setPoseTarget(pose, EE_LINK);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO_STREAM(node->get_logger(), "plan " << (success ? "" : "FAILED"));
    if (success)
    {
      move_group_interface.execute(my_plan);
    }
    else
    {
      return -1;
    }
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Press a key to continue...");
  std::cin >> ans;

  // obj to attach
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = WORLD_FRAME;
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = x1 + DX_0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = Table_DZ;
    // addBOX(planning_scene_interface, 0.02, 0.02, 0.22, pose, "attach_obj");
    std::vector<std::string> touch_links;
    touch_links.push_back("panda_rightfinger");
    touch_links.push_back("panda_leftfinger");
    move_group_interface.attachObject("attach_obj", "panda_hand", touch_links);
  }
  RCLCPP_INFO_STREAM(node->get_logger(), "Press a key to continue...");
  std::cin >> ans;

  gripper_close(node);

  RCLCPP_INFO_STREAM(node->get_logger(), "Press a key to continue...");
  std::cin >> ans;

  // PLAN PLACE
  {
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

    // ADD CONSTRAINTS
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = EE_LINK;
    ocm.header.frame_id = WORLD_FRAME;
    ocm.orientation.x = q.x();
    ocm.orientation.y = q.y();
    ocm.orientation.z = q.z();
    ocm.orientation.w = q.w();
    ocm.absolute_x_axis_tolerance = 2.0 * M_PI;
    ocm.absolute_y_axis_tolerance = 2.0 * M_PI / 180.0;
    ocm.absolute_z_axis_tolerance = 2.0 * M_PI / 180.0;
    ocm.weight = 1.0;
    moveit_msgs::msg::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    // move_group_interface.setPlanningTime(60);
    // move_group_interface.setPathConstraints(test_constraints); // <-- uncomment for orientation constraint

    move_group_interface.setStartStateToCurrentState();  //<-- it is usefull to update the start state to the current
                                                         // state
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = WORLD_FRAME;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose.pose.position.x = 0.0;
    pose.pose.position.y = y1 + Dy_f;
    pose.pose.position.z = Table_DZ + Dz_grasp;

    RCLCPP_INFO_STREAM(node->get_logger(), "Plan target:\n" << geometry_msgs::msg::to_yaml(pose));

    move_group_interface.setPoseTarget(pose, EE_LINK);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO_STREAM(node->get_logger(), "plan " << (success ? "" : "FAILED"));
    if (success)
    {
      move_group_interface.execute(my_plan);
    }
    else
    {
      return -1;
    }

    gripper_open(node);
    move_group_interface.detachObject("attach_obj");
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Press a key to END...");
  std::cin >> ans;

  // Shutdown ROS
  rclcpp::shutdown();
  // We need to join the spinner thread!
  spinner.join();

  return 0;
}
