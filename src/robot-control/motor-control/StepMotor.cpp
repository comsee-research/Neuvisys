//
// Created by thomas on 28/10/2021.
//

#include "StepMotor.hpp"

StepMotor::StepMotor(const std::string &topic, const size_t motorAdress, const std::string &port) : m_motor(static_cast<int>(motorAdress), port) {
    m_topic = topic;
    m_motorSub = nh.subscribe<std_msgs::Float32>(topic, 1000, [this](auto && PH1) {
        setPositionCallBack(std::forward<decltype(PH1)>(PH1)); });

    m_motor.StartDrive();
}

void StepMotor::setPositionCallBack(const ros::MessageEvent<std_msgs::Float32> &position) {
    m_motor.SetAbsolutePosition(static_cast<int>(position.getMessage()->data));
}

StepMotor::~StepMotor() {
    m_motor.StopDrive();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "neuvisysControl");
    if (!ros::master::check()) {
        return 0;
    }
    ros::start();
    ros::Time::init();

    auto stepMotor = StepMotor("leftmotor1", 0, "/dev/ttyUSB0");

    while (ros::ok()) {
        ros::spin();
    }
}