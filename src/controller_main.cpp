#include "../include/controller.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_controller");
	ROS_INFO("Node initialized.");
	ros::NodeHandle nh;

	// Create the controller specific to your robot
	std::shared_ptr<Controller> p_controller = std::make_shared<Controller>(nh, 500.0);
	ROS_INFO("Controller object created.");
	ros::Rate r(500);

	while(ros::ok())
	{
		//auto start = std::chrono::high_resolution_clock::now();
		p_controller->ComputeControlAction();
		ROS_INFO("Control action computed.");
		//auto end = std::chrono::high_resolution_clock::now();
		//double duration_control_action = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;

		ros::spinOnce();
		r.sleep();
	}
	ROS_INFO("Node terminated.");
	return 0;
}
