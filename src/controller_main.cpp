#include "../include/controller.hpp"
#include <chrono>
#include <vector>
#include <numeric>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_controller");
    ROS_INFO("Node initialized.");
    ros::NodeHandle nh;

    // Create the controller specific to your robot
    std::shared_ptr<Controller> p_controller = std::make_shared<Controller>(nh, 500.0);
    ROS_INFO("Controller object created.");

    ros::Rate r(500); // Target loop rate
    auto last_time = std::chrono::high_resolution_clock::now();
    auto start_time = std::chrono::high_resolution_clock::now(); // For periodic logging
    std::vector<double> frequencies; // Store frequencies for averaging

    while (ros::ok())
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        double loop_time = std::chrono::duration<double>(current_time - last_time).count(); // Loop time in seconds
        last_time = current_time;

        double frequency = 1.0 / loop_time; // Compute frequency in Hz
        frequencies.push_back(frequency);

        // Periodically log the average frequency (every 10 seconds)
        if (std::chrono::duration<double>(current_time - start_time).count() >= 10.0)
        {
            double avg_frequency = std::accumulate(frequencies.begin(), frequencies.end(), 0.0) / frequencies.size();
            ROS_INFO("Average Control Loop Frequency: %.2f Hz", avg_frequency);
            frequencies.clear(); // Reset the vector
            start_time = current_time; // Reset the periodic timer
        }

        // Compute control action
        p_controller->ComputeControlAction();
        ROS_INFO("Control action computed.");

        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("Node terminated.");
    return 0;
}
