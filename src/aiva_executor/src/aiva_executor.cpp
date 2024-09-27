#include <cstdio>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <memory>
#include <thread>
void addObjects(moveit::planning_interface::MoveGroupInterface &move_group_interface){
   auto const collision_object = [frame_id =
                                     move_group_interface.getPlanningFrame()]
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box";
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.04;
    primitive.dimensions[primitive.BOX_Y] = 1;
    primitive.dimensions[primitive.BOX_Z] = 1;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0; 
    box_pose.position.x = 0;
    box_pose.position.y = -0.84;
    box_pose.position.z = 0.52;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  auto const table_collision_object = [frame_id =
                                           move_group_interface.getPlanningFrame()]
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "table";
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1;
    primitive.dimensions[primitive.BOX_Y] = 1;
    primitive.dimensions[primitive.BOX_Z] = 0.01;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0;
    box_pose.position.z = -0.05;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);
  planning_scene_interface.applyCollisionObject(table_collision_object);
}





int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "nova2_group");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
  // auto const draw_trajectory_tool_path =
  //     [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("nova2_group")](
  //         auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  // Set a target Pose with updated values !!!
  auto const target_pose = [node] {
    geometry_msgs::msg::Pose msg;
    if (node->get_parameter("pos").as_int() == 0)
   {
    msg.orientation.x = 0.74403;
    msg.orientation.y = 0.065131;
    msg.orientation.z = -0.073367;
    msg.orientation.w = 0.6609;
    msg.position.x = -0.26759;
    msg.position.y = -0.49364;
    msg.position.z = 0.2959;
   }else{
 msg.orientation.x = 0;
    msg.orientation.y = -0.7;
    msg.orientation.z = 0.7;
    msg.orientation.w = 0;
    msg.position.x = 0.28;
    msg.position.y = -0.4236;
    msg.position.z = 0.5;
   }
   
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create collision object for the robot to avoid
 addObjects(move_group_interface);

  // Create a plan to that target pose
  prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  RCLCPP_INFO(node->get_logger(), "Planning");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    // draw_trajectory_tool_path(plan.trajectory);
    moveit_visual_tools.trigger();
    prompt("Press 'next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  }
  else
  {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}




















// int main(int argc, char **argv){
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//       "aiva",
//       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
//   using moveit::planning_interface::MoveGroupInterface;
//   auto move_group_interface = MoveGroupInterface(node, "nova2_group");

//   move_group_interface.setPlannerId("RRTkConfigDefault");

//   move_group_interface.setGoalPositionTolerance(0.001);
//   move_group_interface.setGoalOrientationTolerance(0.001);
//   move_group_interface.setPlanningTime(5);
//   move_group_interface.setNumPlanningAttempts(10);
//   move_group_interface.setMaxVelocityScalingFactor(0.5);
//   move_group_interface.setMaxAccelerationScalingFactor(0.5);

//   geometry_msgs::msg::Pose pose_goal;


//   if (node->get_parameter("pos").as_int() == 0)
//   {
//     pose_goal.orientation.x = 0.74403;
//     pose_goal.orientation.y = 0.065131;
//     pose_goal.orientation.z = -0.073367;
//     pose_goal.orientation.w = 0.6609;
//     pose_goal.position.x = -0.26759;
//     pose_goal.position.y = -0.49364;
//     pose_goal.position.z = 0.2959;
//     RCLCPP_INFO(node->get_logger(), "Moving to left");
//   }
//   else
//   {
//     pose_goal.orientation.x = -1.1836 - 05;
//     pose_goal.orientation.y = 0.70713;
//     pose_goal.orientation.z = -0.70708;
//     pose_goal.orientation.w = -4.3729e-05;
//     pose_goal.position.x = 0.26648;
//     pose_goal.position.y = -0.51719;
//     pose_goal.position.z = 0.37347;
//     RCLCPP_INFO(node->get_logger(), "Moving to right");
//   }
//   move_group_interface.setPoseTarget(pose_goal);
//   // move_group_interface.setStartStateToCurrentState();

//   // auto statemy = *move_group_interface.getCurrentState();
//   addObjects(move_group_interface);
 
//   // Create a plan to that target pose
//   auto const [success, plan] = [&move_group_interface]
//   {
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//   }();

//   printf("planned\n");

//   std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
//       node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
//   moveit_msgs::msg::DisplayTrajectory display_trajectory;

//   display_trajectory.trajectory_start = plan.start_state;
//   display_trajectory.trajectory.push_back(plan.trajectory);

//   display_publisher->publish(display_trajectory);

//   if (success)
//   {
//     move_group_interface.execute(plan);
//   }
//   else
//   {
//     RCLCPP_ERROR(node->get_logger(), "Planing failed!");
//   }
//   rclcpp::shutdown();
//   return 0;
// }
