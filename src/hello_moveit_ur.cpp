// #include <moveit/move_group_interface/move_group_interface.h>
// #include <rclcpp/qos.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include "std_msgs/msg/string.hpp"
// using std::placeholders::_1;

// const std::string MOVE_GROUP = "ur_manipulator";

// class MoveItFollowTarget : public rclcpp::Node
// {
// public:
//   /// Constructor
//   MoveItFollowTarget();

//   /// Move group interface for the robot
//   moveit::planning_interface::MoveGroupInterface move_group_;
//   /// Subscriber for target pose
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

// private:
//   /// Callback that plans and executes trajectory each time the target pose is changed
//   void target_pose_callback(const std_msgs::msg::String::SharedPtr msg);
// };

// MoveItFollowTarget::MoveItFollowTarget() : Node("minimal_subscriber", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
//                                            move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), MOVE_GROUP)
// {
//   rclcpp::shutdown();
//   subscription_ = this->create_subscription<std_msgs::msg::String>(
//       "topic", 10, std::bind(&MoveItFollowTarget::target_pose_callback, this, _1));

//   RCLCPP_INFO(this->get_logger(), "Initialization successful.");
// }

// void MoveItFollowTarget::target_pose_callback(const std_msgs::msg::String::SharedPtr msg)
// {
//   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
// }

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);

//   auto target_follower = std::make_shared<MoveItFollowTarget>();

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(target_follower);
//   executor.spin();

//   rclcpp::shutdown();
//   return EXIT_SUCCESS;
// }





#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_nm")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      

      auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

      using moveit::planning_interface::MoveGroupInterface;
      auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
      
      // Create a ROS logger
      RCLCPP_INFO(this->get_logger(), "I heard:");

    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      // rclcpp::shutdown();
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const a = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(a);
  rclcpp::shutdown();
  return 0;
}









// int main(int argc, char * argv[])
// {
//   // Initialize ROS and create the Node
//   rclcpp::init(argc, argv);
//   auto const node = std::make_shared<rclcpp::Node>(
//     "hello_moveit",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//   );

//   // Create a ROS logger
//   auto const logger = rclcpp::get_logger("hello_moveit");

//   // Create the MoveIt MoveGroup Interface
// using moveit::planning_interface::MoveGroupInterface;
// auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

// // Set a target Pose
// auto const target_pose = []{
//   geometry_msgs::msg::Pose msg;
//   msg.orientation.w = 1.0;
//   msg.position.x = 0.28;
//   msg.position.y = -0.2;
//   msg.position.z = 0.5;
//   return msg;
// }();
// move_group_interface.setPoseTarget(target_pose);

// // Create a plan to that target pose
// auto const [success, plan] = [&move_group_interface]{
//   moveit::planning_interface::MoveGroupInterface::Plan msg;
//   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//   return std::make_pair(ok, msg);
// }();

// // Execute the plan
// if(success) {
//   move_group_interface.execute(plan);
// } else {
//   RCLCPP_ERROR(logger, "Planing failed!");
// }

//   // Shutdown ROS
//   rclcpp::shutdown();
//   return 0;
// }
