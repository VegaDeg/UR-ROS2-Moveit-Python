#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

const std::string MOVE_GROUP = "ur_manipulator";

class ReceiveAndControl : public rclcpp::Node
{
public:
  // Constructor
  ReceiveAndControl();
  // Move group interface for the robot
  moveit::planning_interface::MoveGroupInterface move_group_;
  // Subscriber for target pose
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;

private:
  /// Callback that plans and executes trajectory to new pose
  void send_goal_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
};

ReceiveAndControl::ReceiveAndControl() : Node("receive_control_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
                                           move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
{
  subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "target_pos", 10, std::bind(&ReceiveAndControl::send_goal_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

void ReceiveAndControl::send_goal_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received goal position: '%f', '%f', '%f'", msg->x, msg->y, msg->z);

  auto const target_pose = [msg]{
    geometry_msgs::msg::Pose send_msg;
    send_msg.orientation.w = 1.0;
    send_msg.position.x = 0.28;
    send_msg.position.y = -0.2;
    send_msg.position.z = 1.1;
    return send_msg;
  }();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = this->move_group_.getPlanningFrame();
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 3;
  primitive.dimensions[primitive.BOX_Y] = 3;
  primitive.dimensions[primitive.BOX_Z] = 0.01;
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.01;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);



  this->move_group_.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [this]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(this->move_group_.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    this->move_group_.execute(plan);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planing failed!");
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto interface = std::make_shared<ReceiveAndControl>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(interface);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

