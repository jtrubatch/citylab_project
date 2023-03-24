#include "direction_service_interface/action/go_to_point.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"
#include <functional>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <math.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <thread>

#define PI 3.14159265;

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
        std::bind(&GoToPoint::handleAccepted, this, _1));

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

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose[0] = msg->pose.pose.position.x;
    current_pose[1] = msg->pose.pose.position.y;
    tf2::Quaternion orientation;
    tf2::convert(msg->pose.pose.orientation, orientation);
    current_pose[2] = tf2::getYaw(orientation) * 180 / PI;
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
    const float x_goal = goal_handle->get_goal()->goal_pos.x;
		const float y_goal = goal_handle->get_goal()->goal_pos.y;
		const float w_goal = goal_handle->get_goal()->goal_pos.z;
		
    auto feedback = std::make_shared<GoTo::Feedback>();
    auto &message = feedback->current_pos;

    auto result = std::make_shared<GoTo::Result>();

    rclcpp::Rate loop_rate(10);
		bool complete = false;
		bool atXY = false;
		bool atW = false;
		bool targetW = false;
		float rotation = 1.5; // degrees for +- of target
		float delta_x;
		float delta_y;
		float delta_w;
		int count = 0;
		float w_target = getTheta(current_pose, x_goal, y_goal);
		while(rclcpp::ok() && !complete)
		{

			if(goal_handle->is_canceling())
			{
				result->status = false;
				goal_handle->canceled(result);
				RCLCPP_INFO(this->get_logger(), "Goal Canceled");
				return;
			}
			
			// Rotate to face goal X Y
			if(abs(w_target - current_pose[2])> rotation && !targetW) 
			{
				RCLCPP_INFO(this->get_logger(), "Rotating to Goal");
				if(w_target > 0)
				{
					cmd.linear.x = 0.0;
					cmd.angular.z = 0.25;
				}
				else if(w_target < 0)
				{
					cmd.linear.x = 0.0;
					cmd.angular.z = -0.25;
				}		
				
			}else if(abs(w_target - current_pose[2])<= rotation)
				{
					targetW = true;
				}		

			// Move to goal X Y
			if(!atXY && targetW)
			{
				RCLCPP_INFO(this->get_logger(), "Moving to Goal");
				cmd.linear.x = 0.075;
				
				w_target = getTheta(current_pose, x_goal, y_goal);
				cmd.angular.z = (w_target - current_pose[2]) * 0.1;
				// Truncate delta values at 2 decimal places
				delta_x = roundf((x_goal - current_pose[0]) * 100) / 100; 
				delta_y = roundf((y_goal - current_pose[1]) * 100) / 100;
				if(delta_x == 0.0 && delta_y == 0.0)
				{
					cmd.linear.x = 0.0;
					cmd.angular.z = 0.0;
					cmd_pub->publish(cmd);	
					atXY = true;
				}				
			}
			// Rotate to Goal W
			if(!atW && atXY)
			{
				RCLCPP_INFO(this->get_logger(), "Rotating to Goal Orientation");
				if(w_goal > current_pose[2])
				{
					cmd.linear.x = 0.0;
					cmd.angular.z = 0.15;
				}
				else if(w_goal < current_pose[2])
				{
					cmd.linear.x = 0.0;
					cmd.angular.z = -0.15;
				}	
				// Truncate delta at 1 decimal place
				delta_w = abs(roundf((w_goal - current_pose[2])*10) / 10);
				if(delta_w <= 0.2)
				{
					cmd.linear.x = 0.0;
					cmd.angular.z = 0.0;
					cmd_pub->publish(cmd);
					atW = true;
				}
	
				
			}
			if(atXY && atW)
			{
				complete = true;
			}		
			cmd_pub->publish(cmd);	
			count++;
			if(count == 10 || complete)
			{
				message.x = current_pose[0];
				message.y = current_pose[1];
				message.z = current_pose[2];
				goal_handle->publish_feedback(feedback);
				count = 0;
			}	
			loop_rate.sleep();
		} // End movement while()
		if(rclcpp::ok())
		{
			result->status = true;
			goal_handle->succeed(result);
			RCLCPP_INFO(this->get_logger(), "Goal Reached");
		}
  } // end execute()

  float getTheta(float xy[], const float &goal_x, const float &goal_y) // Pos angle CCW Neg angle CW
  {
    double x, y;
    x = goal_x - xy[0];
    y = goal_y - xy[1];

    return atan2(y, x) * 180 / PI; // Returns Degrees
  }
}; // end class

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<GoToPoint> server = std::make_shared<GoToPoint>();

	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(server);
	executor.spin();

	rclcpp::shutdown();

	return 0;
}
