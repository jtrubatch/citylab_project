#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "direction_service_interface/srv/get_direction_alt.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <vector>
#include <chrono>


using namespace std::chrono_literals;
using std::placeholders::_1;



class TestService : public rclcpp::Node 
{
public:
	
	TestService() : Node("test_service")
	{
		test_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		rclcpp::SubscriptionOptions opt;
		opt.callback_group = test_group;

		sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&TestService::sensorCallback, this, _1), opt);
		client = this->create_client<direction_service_interface::srv::GetDirectionAlt>("direction_service", rmw_qos_profile_services_default, test_group);
		client_timer = this->create_wall_timer(1000ms, std::bind(&TestService::clientCallback, this), test_group);
	}

private:
	rclcpp::CallbackGroup::SharedPtr test_group;
	rclcpp::TimerBase::SharedPtr client_timer;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
	rclcpp::Client<direction_service_interface::srv::GetDirectionAlt>::SharedPtr client;
	float laser[19];

	void clientCallback()
	{
		auto req = std::make_shared<direction_service_interface::srv::GetDirectionAlt::Request>();
		
		for(int i = 0; i < 19; i++)
		{
			req->laser_data[i] = laser[i];
			RCLCPP_INFO(this->get_logger(), " laser data: %f", req->laser_data[i]);
		}
		
		auto result_future = client->async_send_request(req);
		std::future_status status = result_future.wait_for(2s);
		if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Received response");
			auto result = result_future.get();
    	RCLCPP_INFO(this->get_logger(), "Safe Direction is: %s", result->direction.c_str());
		}

	}
	void sensorCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
	{

		laser[0] = msg->ranges[180];
		laser[1] = msg->ranges[200];
		laser[2] = msg->ranges[220];
		laser[3] = msg->ranges[240];
		laser[4] = msg->ranges[260];
		laser[5] = msg->ranges[280];
		laser[6] = msg->ranges[300];
		laser[7] = msg->ranges[320];
		laser[8] = msg->ranges[340];
		laser[9] = msg->ranges[360];
		laser[10] = msg->ranges[380];
		laser[11] = msg->ranges[400];
		laser[12] = msg->ranges[420];
		laser[13] = msg->ranges[440];
		laser[14] = msg->ranges[460];
		laser[15] = msg->ranges[480];
		laser[16] = msg->ranges[500];
		laser[17] = msg->ranges[520];
		laser[18] = msg->ranges[540];

	}

};




int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

	std::shared_ptr<TestService> test_service_node = std::make_shared<TestService>();
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(test_service_node);
	executor.spin();
  
	rclcpp::shutdown();

  return 0;
	


}