//
// Created by thomas on 28/10/2021.
//

#ifndef NEUVISYS_STEPMOTOR_HPP
#define NEUVISYS_STEPMOTOR_HPP

#include <std_msgs/Float32.h>
#include "ros/ros.h"
#include "libserie/Faulhaber.hpp"

class StepMotor {
    ros::NodeHandle nh;
    ros::Subscriber m_motorSub{};

    std::string m_topic;
    Faulhaber m_motor;

public:
    StepMotor(const std::string &topic, const size_t motorAdress, const std::string &port);
    ~StepMotor();
    void setPositionCallBack(const ros::MessageEvent<std_msgs::Float32> &position);
};


#endif //NEUVISYS_STEPMOTOR_HPP
