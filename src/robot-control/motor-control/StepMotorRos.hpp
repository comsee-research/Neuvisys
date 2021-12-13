//
// Created by thomas on 28/10/2021.
//

#ifndef NEUVISYS_STEPMOTORROS_HPP
#define NEUVISYS_STEPMOTORROS_HPP

#include "BrushlessMotor.hpp"
#include <std_msgs/Float32.h>
#include "ros/ros.h"

class StepMotorRos : public BrushlessMotor {
    ros::NodeHandle m_nh;
    ros::Subscriber m_positionSub{};
    ros::Subscriber m_speedSub{};

public:
    StepMotorRos(const std::string &topic, size_t motorAdress, const std::string &port);
    void setSpeedCallBack(const ros::MessageEvent<std_msgs::Float32> &speed);
    void setPositionCallBack(const ros::MessageEvent<std_msgs::Float32> &position);
};


#endif //NEUVISYS_STEPMOTORROS_HPP
