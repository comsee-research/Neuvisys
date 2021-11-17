//
// Created by thomas on 04/06/2021.
//

#include "Motor.hpp"

Motor::Motor(ros::NodeHandle &n, const std::string& name) {
    m_motorPub = n.advertise<std_msgs::Float32>(name, 1000);
}

void Motor::jitter(double dt, double jitter) {
    if (jitter == 0) {
        m_jitterPos = Util::ornsteinUhlenbeckProcess(dt, m_jitterPos, 25, 0., 0.05);
    } else {
        m_jitterPos = jitter;
    }
    m_speed.data = static_cast<float>(m_jitterPos);
    m_motorPub.publish(m_speed);
}

void Motor::move() {
    m_motorPub.publish(m_speed);
}

void Motor::incrementSpeed(float increment) {
    m_speed.data += increment;
}