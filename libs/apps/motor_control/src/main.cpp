//
// Created by Thomas on 28/10/2021.
//

#include <motor_control/MotorRos.hpp>

#include <chrono>
#include <string>


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "neuvisysControl");
    if (!ros::master::check()) {
        std::cout << "ROS not launched" << std::endl;
        return 0;
    }
    ros::start();

    std::chrono::high_resolution_clock::time_point time;
    std::chrono::high_resolution_clock::time_point timePosition = std::chrono::high_resolution_clock::now();

    auto stepMotor = MotorRos("leftmotor1", 0, "/dev/ttyUSB0");
    stepMotor.setSpeed(100);

    double position = stepMotor.getPosition();
    std::cout << position << std::endl;
    while (ros::ok() && position < 3000) {
        ros::spinOnce();
        time = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration_cast<std::chrono::microseconds>(time - timePosition).count() > 100000) {
            timePosition = std::chrono::high_resolution_clock::now();

            position = stepMotor.getPosition();
            std::cout << position << std::endl;
        }
    }
    return 0;
}
