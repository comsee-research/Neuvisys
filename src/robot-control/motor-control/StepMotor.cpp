//
// Created by thomas on 19/11/2021.
//

#include "StepMotor.hpp"

StepMotor::StepMotor(const std::string &topic, const size_t motorAdress, const std::string &port) : m_motor(static_cast<int>(motorAdress), port) {
    m_motor.StartDrive();
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

double StepMotor::getPosition() {
    return m_motor.GetPosition();
}

bool StepMotor::isActionValid(double position, double projection) {
    auto projectedPosition = position + projection;
    if (projectedPosition < m_lowerBound || projectedPosition > m_upperBound) {
        return false;
    }
    return true;
}