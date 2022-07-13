//
// Created by Thomas on 28/10/2021.
//

#include <chrono>
#include <string>

#include "MotorRos.hpp"

MotorRos::MotorRos(const std::string &topic, const size_t motorAdress, const std::string &port) : BrushlessMotor(motorAdress, port) {
    m_positionSub = m_nh.subscribe<std_msgs::Float32>(topic + "Speed", 1000, [this](auto && PH1) {
        setSpeedCallBack(std::forward<decltype(PH1)>(PH1)); });
    m_speedSub = m_nh.subscribe<std_msgs::Float32>(topic + "Position", 1000, [this](auto && PH1) {
        setPositionCallBack(std::forward<decltype(PH1)>(PH1)); });
}

void MotorRos::setSpeedCallBack(const ros::MessageEvent<std_msgs::Float32> &speed) {
    m_motor.SetSpeed(static_cast<int>(speed.getMessage()->data));
}

void MotorRos::setPositionCallBack(const ros::MessageEvent<std_msgs::Float32> &position) {
    m_motor.SetAbsolutePosition(static_cast<int>(position.getMessage()->data));
}
