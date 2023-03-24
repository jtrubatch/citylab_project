#include "direction_service_interface/action/detail/go_to_point__struct.hpp"
#include "direction_service_interface/action/go_to_point.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"
#include <functional>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <math.h>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class GoToPoint : public rclcpp::Node {
public:
  using GoTo = direction_service_interface::action::GoToPoint;
  using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoTo>;

  explicit GoToPoint(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("action_server") {
    action_group =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions action_opt;

    this->action_server = rclcpp_action::create_server<GoTo>(
        this, "go_to_point", std::bind(&GoToPoint::handleGoal, this, _1, _2),
        std::bind(&GoToPoint::handleCancel, this, _1),
        std::bind(&GoToPoint::handleAccepted, this, _1), action_group);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&GoToPoint::odomCallback, this, _1), action_opt);

    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
  rclcpp_action::Server<GoTo>::SharedPtr action_server;
  rclcpp::CallbackGroup::SharedPtr action_group;
  geometry_msgs::msg::Twist cmd;
	float current_pose[3] = {0, 0, 0}; // x, y, w
	float w_goal;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
	{
		current_pose[0] = msg->pose.pose.position.x;
		current_pose[1] = msg->pose.pose.position.y;
		tf2::Quaternion orientation;
		tf2::convert(msg->pose.pose.orientation, orientation);
		current_pose[2] = tf2::getYaw(orientation);
	}

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const GoTo::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandleGoTo> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleGoTo> goal_handle) {

    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPoint::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoTo> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Move::Feedback>();
    auto &message = feedback->feedback;
    message = "Starting movement...";
    auto result = std::make_shared<Move::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);
  }

	float getTheta(float xy[], float &goal_x, float &goal_y) // Pos angle CCW Neg angle CW
	{

		
	}
};