#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "direction_service_interface/srv/get_direction.hpp"
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
		client = this->create_client<direction_service_interface::srv::GetDirection>("direction_service", rmw_qos_profile_services_default, patrol_group);
		cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);

		control_timer = this->create_wall_timer(
											200ms, std::bind(&Patrol::commandPublisher, this), patrol_group);
		
		msg_timer = this->create_wall_timer(
									200ms, std::bind(&Patrol::infoMsg, this), patrol_group);
	
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
		auto req = std::make_shared<direction_service_interface::srv::GetDirection::Request>();
		req->laser_data.ranges = msg->ranges;

		auto result_future = client->async_send_request(req);
		std::future_status status = result_future.wait_for(2s);
		if (status == std::future_status::ready) 
		{
		auto result = result_future.get();
    RCLCPP_INFO(this->get_logger(), "Safe Direction is: %s", result->direction.c_str());
		}
		direction = result_future.get()->direction;

	}
	// Service client callback and command publisher
  void commandPublisher()  // positive CCW negative CW
	{	
		if(direction == "forward")
		{
			cmd.linear.x = 0.05;
			cmd.angular.z = 0.0;
		}
		else if(direction == "left")		
		{
			cmd.linear.x = 0.01;
			cmd.angular.z = 0.35;
		}
		else if(direction == "right")		
		{
			cmd.linear.x = 0.01;
			cmd.angular.z = -0.35;
		}else
		{
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.0;
			RCLCPP_INFO(this->get_logger(), "No Direction Given. Stopping");
		
		}
		this->cmd_pub->publish(cmd);
	}

	void infoMsg()
	{
		RCLCPP_INFO(this->get_logger(), "Linear Velocity: %f   Angular Velocity %f", cmd.linear.x, cmd.angular.z);
	}

	rclcpp::CallbackGroup::SharedPtr patrol_group;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
	rclcpp::Client<direction_service_interface::srv::GetDirection>::SharedPtr client;
	
	rclcpp::TimerBase::SharedPtr control_timer;
	rclcpp::TimerBase::SharedPtr msg_timer;
  
	geometry_msgs::msg::Twist cmd;
	std::string direction;
	
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