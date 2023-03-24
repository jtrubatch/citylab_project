#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/subscription_options.hpp"
#include <algorithm>
#include <ratio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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
    // Data from 180 to 540 increments of 20.
		std::vector<float> right_side = {
            msg->ranges[180], msg->ranges[200], msg->ranges[220], 
            msg->ranges[240], msg->ranges[260], msg->ranges[280]  
    };    
		std::vector<float> front = { // 9 is Center/360
            msg->ranges[300], msg->ranges[320], msg->ranges[340], 
            msg->ranges[360], msg->ranges[380], msg->ranges[400], msg->ranges[420]
    };
    std::vector<float> left_side = {
            msg->ranges[440], msg->ranges[460], msg->ranges[480], 
            msg->ranges[500], msg->ranges[520], msg->ranges[540]
    };

		left_min = *min_element(left_side.begin(),left_side.end());
		left_max = *max_element(left_side.begin(),left_side.end());
		right_min = *min_element(right_side.begin(),right_side.end());
		right_max = *max_element(right_side.begin(),right_side.end());
		front_min = *min_element(front.begin(), front.end());
    
	}

  void commandPublisher()  // positive CCW negative CW
	{	
    float linear_vel = 0.15;

    if(left_min < side_min && front_min > min_distance)
		{
			cmd.linear.x = linear_vel * 0.5;
		  cmd.angular.z = -0.15;
		}else if(right_min < side_min && front_min > min_distance)
		{
			cmd.linear.x = linear_vel * 0.5;
		  cmd.angular.z = 0.15;
		}else if(front_min > min_distance)
		{
			cmd.linear.x = linear_vel;
		  cmd.angular.z = 0.0;

		}else if(front_min < min_distance) 
		{
			if(compareSidesLeft(left_max, right_max))
			{
				cmd.linear.x = linear_vel * 0.25;
		    cmd.angular.z = 0.35;

			}
			else if(!compareSidesLeft(left_max, right_max))
			{
				cmd.linear.x = linear_vel * 0.25;
		    cmd.angular.z = -0.35;

			}		
    } 

		this->cmd_pub->publish(cmd);
	}

  bool compareSidesLeft(float const &left, float const &right)
	{
		if(left >= right)
		{
			return true;
		}
		else 
		{
			return false;
		}	
	}

	void infoMsg()
	{
		RCLCPP_INFO(this->get_logger(), "Linear Velocity: %f   Angular Velocity %f", cmd.linear.x, cmd.angular.z);
	}

	
	rclcpp::CallbackGroup::SharedPtr patrol_group;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
	rclcpp::TimerBase::SharedPtr control_timer;
	rclcpp::TimerBase::SharedPtr msg_timer;
	geometry_msgs::msg::Twist cmd;
	
	float left_min = 0.0;
	float left_max = 0.0;
  float front_min = 0.0;
  float right_min = 0.0;
	float right_max = 0.0;
	const float min_distance = 0.5;
	const float side_min = 0.25;
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