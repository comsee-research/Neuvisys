//
// Created by thomas on 28/10/2021.
//

#include "StepMotor.hpp"

StepMotor::StepMotor(const std::string &topic, const size_t motorAdress, const std::string &port) : m_motor(static_cast<int>(motorAdress), port) {
    m_positionSub = m_nh.subscribe<std_msgs::Float32>(topic + "Speed", 1000, [this](auto && PH1) {
        setSpeedCallBack(std::forward<decltype(PH1)>(PH1)); });
    m_speedSub = m_nh.subscribe<std_msgs::Float32>(topic + "Position", 1000, [this](auto && PH1) {
        setPositionCallBack(std::forward<decltype(PH1)>(PH1)); });

    m_motor.StartDrive();
}

void StepMotor::setSpeedCallBack(const ros::MessageEvent<std_msgs::Float32> &position) {
    m_motor.SetSpeed(static_cast<int>(position.getMessage()->data));
}

void StepMotor::setPositionCallBack(const ros::MessageEvent<std_msgs::Float32> &position) {
    m_motor.SetAbsolutePosition(static_cast<int>(position.getMessage()->data));
}

void StepMotor::setSpeed(int speed) {
    m_motor.SetSpeed(speed);
}

void StepMotor::setPosition(int position) {
    m_motor.SetAbsolutePosition(position);
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

//    stepMotor.setSpeed(100);

    while (ros::ok()) {
        ros::spin();
    }
}