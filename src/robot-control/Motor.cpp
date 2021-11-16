//
// Created by thomas on 04/06/2021.
//

#include "Motor.hpp"

Motor::Motor(ros::NodeHandle &n, const std::string& name) {
    m_motorPub = n.advertise<std_msgs::Float32>(name, 1000);
}

void Motor::jitter(double dt) {
    OrnsteinUhlenbeckProcess(dt, 25, 0., 0.05);
    m_speed.data = static_cast<float>(x);
    m_motorPub.publish(m_speed);
}

void Motor::OrnsteinUhlenbeckProcess(double dt, double theta, double mu, double sigma) {
    double noise = distribution(generator) * std::sqrt(dt);
    x = x + theta * (mu - x) * dt + sigma * noise;
}

void Motor::move() {
    m_motorPub.publish(m_speed);
}

void Motor::incrementSpeed(float increment) {
    m_speed.data += increment;
}