//
// Created by thomas on 04/06/2021.
//

#ifndef NEUVISYS_MOTOR_H
#define NEUVISYS_MOTOR_H

#include <std_msgs/Float32.h>
#include "ros/ros.h"
#include <cstdlib>
#include <cmath>
#include <random>

class Motor {
    ros::Publisher motorPub{};
    double x = 0;
    std_msgs::Float32 position{};

    std::random_device r;
    std::default_random_engine generator = std::default_random_engine(r());
    std::normal_distribution<double> distribution = std::normal_distribution<double>(0.0, 1.0);

public:
    Motor(ros::NodeHandle &n, const std::string& name);
    void jitter(double dt);
    void move(double pos);
private:
    void OrnsteinUhlenbeckProcess(double dt, double theta, double mu, double sigma);
};

#endif //NEUVISYS_MOTOR_H