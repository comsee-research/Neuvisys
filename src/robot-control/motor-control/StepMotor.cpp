//
// Created by thomas on 19/11/2021.
//

#include "StepMotor.hpp"

StepMotor::StepMotor(const std::string &topic, const size_t motorAdress, const std::string &port) : m_motor(static_cast<int>(motorAdress), port) {
    m_motor.StartDrive();
}

void StepMotor::setSpeed(int speed) {
    m_speed = speed;
    m_motor.SetSpeed(speed);
}

void StepMotor::setPosition(int position) {
    m_pos = position;
    m_motor.SetAbsolutePosition(position);
}

StepMotor::~StepMotor() {
    m_motor.StopDrive();
}

double StepMotor::getPosition() {
    return m_motor.GetPosition();
}

bool StepMotor::isActionValid(double position, double projection) const {
    auto projectedPosition = position + projection;
    if (projectedPosition < m_lowerBound || projectedPosition > m_upperBound) {
        return false;
    }
    return true;
}

void StepMotor::jitterPos(double dt) {
    Util::ornsteinUhlenbeckProcess(m_jitterPos, dt, 25, 0, 0.05);
    setPosition(static_cast<int>(m_pos + m_jitterPos));
}

void StepMotor::jitterSpeed(double dt) {
    Util::ornsteinUhlenbeckProcess(m_jitterSpeed, dt, 25, 0, 0.05);
    setSpeed(static_cast<int>(m_speed + m_jitterSpeed));
}