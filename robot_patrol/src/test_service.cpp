#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "direction_service_interface/srv/get_direction.hpp"
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
		client = this->create_client<direction_service_interface::srv::GetDirection>("direction_service", rmw_qos_profile_services_default, test_group);   
	}

private:
	rclcpp::CallbackGroup::SharedPtr test_group;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
	rclcpp::Client<direction_service_interface::srv::GetDirection>::SharedPtr client;
	
	void sensorCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
	{
		rclcpp::Rate rate(10);
		auto req = std::make_shared<direction_service_interface::srv::GetDirection::Request>();
		req->laser_data.ranges = msg->ranges;

		auto result_future = client->async_send_request(req);
		std::future_status status = result_future.wait_for(2s);
		if (status == std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Received response");
		auto result = result_future.get();
    RCLCPP_INFO(this->get_logger(), "Safe Direction is: %s", result->direction.c_str());
		}
		rate.sleep();
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