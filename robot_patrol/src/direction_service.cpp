#include <rclcpp/rclcpp.hpp>
#include "direction_service_interface/srv/get_direction.hpp"
#include <numeric>

using GetDirection = direction_service_interface::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node 
{
public:
	DirectionService() : Node("direction_server")
	{
		direction_server = create_service<GetDirection>(
												"direction_service", std::bind(&DirectionService::directionCallback, this, _1, _2));	
	}

private:
	rclcpp::Service<GetDirection>::SharedPtr direction_server;
	float left_avg = 0.0;
  float front_avg = 0.0;
  float right_avg = 0.0;
	const float min_distance = 0.35;


	void directionCallback(const std::shared_ptr<GetDirection::Request> req,
												 const std::shared_ptr<GetDirection::Response> res)
	{   
    // Data from 180 to 540 increments of 20.
		std::vector<float> right_side = {
            req->laser_data.ranges[0], req->laser_data.ranges[1], req->laser_data.ranges[2], 
            req->laser_data.ranges[3], req->laser_data.ranges[4], req->laser_data.ranges[5], req->laser_data.ranges[6]   
    };    
		std::vector<float> front = { // 9 is Center/360
            req->laser_data.ranges[7], req->laser_data.ranges[8], 
            req->laser_data.ranges[9], req->laser_data.ranges[10], req->laser_data.ranges[11]
    };
    std::vector<float> left_side = {
            req->laser_data.ranges[12], req->laser_data.ranges[13], req->laser_data.ranges[14], 
            req->laser_data.ranges[15], req->laser_data.ranges[16], req->laser_data.ranges[17], req->laser_data.ranges[18]
    };

		left_avg = getAverage(left_side);
		right_avg = getAverage(right_side);
		front_avg = getAverage(front);
		// At obstacle seek max opening
		if(front_avg <= min_distance)
		{
			if(compareSidesLeft(left_avg, right_avg))
			{
				res->direction = "left";
				RCLCPP_INFO(this->get_logger(), "Safe Direction Left");
			}
			else if(!compareSidesLeft(left_avg, right_avg))
			{
				res->direction = "right";
				RCLCPP_INFO(this->get_logger(), "Safe Direction Right");
			}
		
		}
		else if(front_avg > min_distance)
		{
			res->direction = "forward";
			RCLCPP_INFO(this->get_logger(), "Safe Direction Forward");
		}

	}

	float getAverage(std::vector<float> const &section)
	{
		return std::accumulate(section.begin(), section.end(), 0.0) / section.size();
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
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DirectionService>());
	rclcpp::shutdown();

	return 0;

}