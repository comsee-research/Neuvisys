//
// Created by thomas on 28/10/2021.
//

#include "StepMotorRos.hpp"
#include <string>

StepMotorRos::StepMotorRos(const std::string &topic, const size_t motorAdress, const std::string &port) : StepMotor(topic, motorAdress, port) {
    m_positionSub = m_nh.subscribe<std_msgs::Float32>(topic + "Speed", 1000, [this](auto && PH1) {
        setSpeedCallBack(std::forward<decltype(PH1)>(PH1)); });
    m_speedSub = m_nh.subscribe<std_msgs::Float32>(topic + "Position", 1000, [this](auto && PH1) {
        setPositionCallBack(std::forward<decltype(PH1)>(PH1)); });
}

void StepMotorRos::setSpeedCallBack(const ros::MessageEvent<std_msgs::Float32> &speed) {
    m_motor.SetSpeed(static_cast<int>(speed.getMessage()->data));
}

void StepMotorRos::setPositionCallBack(const ros::MessageEvent<std_msgs::Float32> &position) {
    m_motor.SetAbsolutePosition(static_cast<int>(position.getMessage()->data));
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "neuvisysControl");
    if (!ros::master::check()) {
        std::cout << "ROS not launched" << std::endl;
        return 0;
    }
    ros::start();

    std::chrono::high_resolution_clock::time_point time;
    std::chrono::high_resolution_clock::time_point timePosition = std::chrono::high_resolution_clock::now();

    auto stepMotor = StepMotorRos("leftmotor1", 0, "/dev/ttyUSB0");
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
}