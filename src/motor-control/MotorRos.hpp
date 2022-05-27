//
// Created by Thomas on 28/10/2021.
//

#ifndef NEUVISYS_MOTORROS_HPP
#define NEUVISYS_MOTORROS_HPP

#include "BrushlessMotor.hpp"
#include <std_msgs/Float32.h>
#include "ros/ros.h"

class MotorRos : public BrushlessMotor {
    ros::NodeHandle m_nh;
    ros::Subscriber m_positionSub{};
    ros::Subscriber m_speedSub{};

public:
    MotorRos(const std::string &topic, size_t motorAdress, const std::string &port);
    void setSpeedCallBack(const ros::MessageEvent<std_msgs::Float32> &speed);
    void setPositionCallBack(const ros::MessageEvent<std_msgs::Float32> &position);
};


#endif //NEUVISYS_MOTORROS_HPP
