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
		direction_server = create_service<GetDirection>("direction_service", std::bind(&DirectionService::directionCallback, this, _1, _2));	
	}

private:
	rclcpp::Service<GetDirection>::SharedPtr direction_server;
	float left_min = 0.0;
	float left_max = 0.0;
  float front_min = 0.0;
  float right_min = 0.0;
	float right_max = 0.0;
	const float min_distance = 0.35;
	const float side_min = 0.2;


	void directionCallback(const std::shared_ptr<GetDirection::Request> req,
												 const std::shared_ptr<GetDirection::Response> res)
	{   
    // Data from 180 to 540 increments of 20.
		std::vector<float> right_side = {
            req->laser_data.ranges[180], req->laser_data.ranges[200], req->laser_data.ranges[220], 
            req->laser_data.ranges[240], req->laser_data.ranges[260], req->laser_data.ranges[280]  
    };    
		std::vector<float> front = { // 9 is Center/360
            req->laser_data.ranges[300], req->laser_data.ranges[320], req->laser_data.ranges[340], 
            req->laser_data.ranges[360], req->laser_data.ranges[380], req->laser_data.ranges[400], req->laser_data.ranges[420]
    };
    std::vector<float> left_side = {
            req->laser_data.ranges[440], req->laser_data.ranges[460], 
            req->laser_data.ranges[480], req->laser_data.ranges[500], req->laser_data.ranges[520], req->laser_data.ranges[540]
    };

		left_min = *min_element(left_side.begin(),left_side.end());
		left_max = *max_element(left_side.begin(),left_side.end());
		right_min = *min_element(right_side.begin(),right_side.end());
		right_max = *max_element(right_side.begin(),right_side.end());
		front_min = *min_element(front.begin(), front.end());
    RCLCPP_INFO(this->get_logger(), "L Min: %f  R Min: %f F Min: %f", left_min, right_min, front_min);
		RCLCPP_INFO(this->get_logger(), "L Max: %f  R Max: %f ", left_max, right_max);
		
	  if(left_min < side_min && front_min > min_distance)
		{
			res->direction = "right";
		}else if(right_min < side_min && front_min > min_distance)
		{
			res->direction = "left";
		}else if(front_min > min_distance)
		{
			res->direction = "forward";

		}else if(front_min < min_distance) 
		{
			if(compareSidesLeft(left_max, right_max))
			{
				res->direction = "left";

			}
			else if(!compareSidesLeft(left_max, right_max))
			{
				res->direction = "right";

			}		
		}
	
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