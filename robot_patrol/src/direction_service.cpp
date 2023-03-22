#include <rclcpp/rclcpp.hpp>
#include "direction_service_interface/srv/get_direction_alt.hpp"
#include <numeric>

using GetDirection = direction_service_interface::srv::GetDirectionAlt;
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
	const float min_distance = 0.25;


	void directionCallback(const std::shared_ptr<GetDirection::Request> req,
												 const std::shared_ptr<GetDirection::Response> res)
	{   
    // Data from 180 to 540 increments of 20.
		std::vector<float> right_side = {
            req->laser_data[0], req->laser_data[1], req->laser_data[2], 
            req->laser_data[3], req->laser_data[4], req->laser_data[5], req->laser_data[6]   
    };    
		std::vector<float> front = { // 9 is Center/360
            req->laser_data[7], req->laser_data[8], 
            req->laser_data[9], req->laser_data[10], req->laser_data[11]
    };
    std::vector<float> left_side = {
            req->laser_data[12], req->laser_data[13], req->laser_data[14], 
            req->laser_data[15], req->laser_data[16], req->laser_data[17], req->laser_data[18]
    };

		left_avg = getAverage(left_side);
		right_avg = getAverage(right_side);
		front_avg = getAverage(front);
        RCLCPP_INFO(this->get_logger(), "leftavg: %f  rightavg: %f frontavg: %f ", left_avg, right_avg, front_avg);
		// At obstacle seek max opening
	    if(front_avg > min_distance)
		{
			res->direction = "forward";
			RCLCPP_INFO(this->get_logger(), "Safe Direction Forward");
		}   
        else
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