#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/subscription_options.hpp"
#include <ratio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Patrol : public rclcpp::Node 
{
public:  
  Patrol() : Node("patrol_node")
  {
		patrol_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
		rclcpp::SubscriptionOptions patrol_opt;
		patrol_opt.callback_group = patrol_group;
		scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
								"scan", 10, std::bind(&Patrol::sensorCallback, this, _1), patrol_opt);
		
		cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);
	
		control_timer = this->create_wall_timer(
											250ms, std::bind(&Patrol::commandPublisher, this), patrol_group);
	
		cmd.linear.x = 0.0;
		cmd.angular.z = 0.0;
  }

private:
  void sensorCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)  // 0/720 Rear  180 Right 360 Front 540 Left
	{
		this->front = msg->ranges[360];
		for(int i = 0; i < 135; i++)
		{
			left_side[i] = msg->ranges[405+i];
			right_side[i] = msg->ranges[360+i];
		}
		left_max = *std::max_element(left_side.begin(), left_side.end());
		right_max = *std::max_element(right_side.begin(), right_side.end());
	}
  void commandPublisher()  // positive CCW negative CW
	{
		if(front <= obstacle_buffer)
		{
			//Opening on Left
			if(left_max > right_max)
			{
				//Turn Left
				rotateLeft(left_max);
			}
			//Opening on Right or Same both sides
			else if(right_max >= left_max)
			{
				//Turn Right
				rotateRight(right_max);
			}
		}
		else 
		{
			RCLCPP_INFO(this->get_logger(), "Moving Forward");
			cmd.angular.z = 0.0;
			cmd.linear.x = 0.25;
		}
		cmd_pub->publish(cmd);

	}

	void rotateRight(float max)
	{
		bool turn_complete = false;
		RCLCPP_INFO(this->get_logger(), "ROTATING RIGHT");
		
		while(!turn_complete)
		{
			cmd.angular.z = -0.25;
			
			if(front >= (max - max_buffer))
			{
				turn_complete = true;
				cmd.angular.z = 0.0;
			}	
			cmd_pub->publish(cmd);	
		}
		RCLCPP_INFO(this->get_logger(), "ROTATION COMPLETE");
	}

	void rotateLeft(float max)
	{
		bool turn_complete = false;
		RCLCPP_INFO(this->get_logger(), "ROTATING LEFT");
		
		while(!turn_complete)
		{
			cmd.angular.z = 0.25;
			
			if(front >= (max - max_buffer))
			{
				turn_complete = true;
				cmd.angular.z = 0.0;
			}	
			cmd_pub->publish(cmd);	
		}
		RCLCPP_INFO(this->get_logger(), "ROTATION COMPLETE");	
	}
	rclcpp::CallbackGroup::SharedPtr patrol_group;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
	rclcpp::TimerBase::SharedPtr control_timer;
  
	float front;
	std::vector<float> left_side;
	float left_max;
	std::vector<float> right_side;
	float right_max;
	geometry_msgs::msg::Twist cmd;
	
	float obstacle_buffer = 0.35;
	float max_buffer = 0.1;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	std::shared_ptr<Patrol> patrol_node = std::make_shared<Patrol>();

	rclcpp::executors::MultiThreadedExecutor execute;
	execute.add_node(patrol_node);
	execute.spin();

	rclcpp::shutdown();

	return 0;
}