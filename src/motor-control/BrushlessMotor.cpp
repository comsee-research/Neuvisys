//
// Created by thomas on 19/11/2021.
//

#include "BrushlessMotor.hpp"

BrushlessMotor::BrushlessMotor(const size_t motorAdress, const std::string &port) : m_motor(static_cast<int>(motorAdress), port) {
    m_motor.StartDrive();
}

void BrushlessMotor::setSpeed(int speed) {
    m_speed = speed;
    m_motor.SetSpeed(speed);
}

void BrushlessMotor::setPosition(int position) {
    m_pos = position;
    m_motor.SetAbsolutePosition(position);
}

BrushlessMotor::~BrushlessMotor() {
    m_motor.StopDrive();
}

double BrushlessMotor::getPosition() {
    return m_motor.GetPosition();
}

bool BrushlessMotor::isActionValid(double position, double projection) const {
    auto projectedPosition = position + projection;
    if (projectedPosition < m_lowerBound || projectedPosition > m_upperBound) {
        return false;
    }
    return true;
}

void BrushlessMotor::jitterPos(double dt) {
    Util::ornsteinUhlenbeckProcess(m_jitterPos, dt, 25, 0, 0.05);
    m_motor.SetAbsolutePosition(static_cast<int>(m_pos + m_jitterPos));
}

void BrushlessMotor::jitterSpeed(double dt) {
    Util::ornsteinUhlenbeckProcess(m_jitterSpeed, dt, 25, 0, 0.05);
    m_motor.SetSpeed(static_cast<int>(m_speed + m_jitterSpeed));
}