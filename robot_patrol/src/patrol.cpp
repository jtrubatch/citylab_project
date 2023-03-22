#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/subscription_options.hpp"
#include <algorithm>
#include <ratio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <chrono>
#include <iterator>

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
											100ms, std::bind(&Patrol::commandPublisher, this), patrol_group);
		
		msg_timer = this->create_wall_timer(
									500ms, std::bind(&Patrol::infoMsg, this), patrol_group);
	
		cmd.linear.x = 0.0;
		cmd.angular.z = 0.0;
    
  }
	~Patrol()
	{
		RCLCPP_INFO(this->get_logger(), "Destructor Called");
	}
private:
  void sensorCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)  // 0/720 Rear  180 Right 360 Front 540 Left
	{
		
		this->front = msg->ranges[360];
		this->laser = {msg->ranges[180], msg->ranges[225], msg->ranges[270], msg->ranges[315], msg->ranges[360],
									 msg->ranges[405], msg->ranges[450], msg->ranges[495], msg->ranges[540]};

		auto min_iterator = std::min_element(laser.begin(), laser.end());
		this->laser_min_index = std::distance(laser.begin(), min_iterator);
		this->laser_min = laser[laser_min_index];
	}
  void commandPublisher()  // positive CCW negative CW
	{	
		this->cmd_pub->publish(cmd);

		cmd.linear.x = linearControl(front);
		cmd.angular.z = angularControl(laser_min, laser_min_index );

		this->cmd_pub->publish(cmd);

	}

	void infoMsg()
	{
		RCLCPP_INFO(this->get_logger(), "Linear Velocity: %f   Angular Velocity %f", cmd.linear.x, cmd.angular.z);
		RCLCPP_INFO(this->get_logger(), "Front Laser: %f   Min Index: %d  Min Value: %f", front, laser_min_index, laser_min);
	}

	float linearControl(float distance) // 0 to 0.75
	{
		float clamped_front;
		if(distance > 0.75)
		{
			clamped_front = 0.75;
		}
		else if(distance < min_distance)
		{
			clamped_front = min_distance;
		}
		else 
		{
			clamped_front = distance;
		}

		return (clamped_front - min_distance) * 0.25;
	}

	float angularControl(float range, int index)
	{
		float vel = 0.0;
		if(range <= obstacle_buffer)
		{
			if(index < 4 ) // Object on right, turn left
			{
				vel = 0.2;
			}
			else if(index >= 4) //Object on left or straight ahead, turn right
			{
				vel = -0.2;
			}

		}
		return vel;
	}
	rclcpp::CallbackGroup::SharedPtr patrol_group;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
	rclcpp::TimerBase::SharedPtr control_timer;
	rclcpp::TimerBase::SharedPtr msg_timer;
  
	float front;
	std::vector <float> laser;
	float laser_min;
	int laser_min_index;
	geometry_msgs::msg::Twist cmd;
	
	
	float obstacle_buffer = 0.35;
	float min_distance = 0.25;
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