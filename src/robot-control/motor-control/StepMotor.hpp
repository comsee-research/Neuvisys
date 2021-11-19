//
// Created by thomas on 28/10/2021.
//

#ifndef NEUVISYS_STEPMOTOR_HPP
#define NEUVISYS_STEPMOTOR_HPP

#include <std_msgs/Float32.h>
#include "ros/ros.h"
#include "libserie/Faulhaber.hpp"

class StepMotor {
    ros::NodeHandle m_nh;
    ros::Subscriber m_positionSub{};
    ros::Subscriber m_speedSub{};
    Faulhaber m_motor;

public:
    StepMotor(const std::string &topic, size_t motorAdress, const std::string &port);
    ~StepMotor();
    void setSpeedCallBack(const ros::MessageEvent<std_msgs::Float32> &speed);
    void setPositionCallBack(const ros::MessageEvent<std_msgs::Float32> &position);
    void setSpeed(int speed);
    void setPosition(int position);
    double getPosition();
};


#endif //NEUVISYS_STEPMOTOR_HPP
