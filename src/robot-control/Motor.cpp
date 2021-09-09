//
// Created by thomas on 04/06/2021.
//

#include "Motor.hpp"

Motor::Motor(ros::NodeHandle &n, const std::string& name) {
    motorPub = n.advertise<std_msgs::Float32>(name, 1000);
}

void Motor::jitter(double dt) {
    OrnsteinUhlenbeckProcess(dt, 1.1, 0., 0.01);
    position.data = x;
    motorPub.publish(position);
}

void Motor::OrnsteinUhlenbeckProcess(double dt, double theta, double mu, double sigma) {
    double noise = distribution(generator) * std::sqrt(dt);
    x = x + theta * (mu - x) * dt + sigma * noise;
}

void Motor::move(double pos) {
    position.data = pos;
    motorPub.publish(position);
}
