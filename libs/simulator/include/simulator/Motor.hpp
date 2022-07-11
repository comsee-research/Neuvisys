//
// Created by Thomas on 04/06/2021.
//

#ifndef NEUVISYS_MOTOR_H
#define NEUVISYS_MOTOR_H

//std
#include <cstdlib>

//ros
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

//neuvisys
#include <network/Util.hpp>

class Motor {
    ros::Publisher m_motorSpeedPub{};
    ros::Publisher m_motorPositionPub{};
    double m_jitterSpeed = 0;
    double m_motion = 0;
    std_msgs::Float32 m_speed{};
    std_msgs::Float32 m_position{};

public:
    Motor(ros::NodeHandle &n, const std::string &topic);
    void jitter(double dt, double jitter = 0);
    void changeSpeed(float speed);
    void changePosition(float position);

    [[nodiscard]] float getSpeed() const { return m_speed.data; }
    [[nodiscard]] float getPosition() const { return m_position.data; }
    [[nodiscard]] double getJitterPos() const { return m_jitterSpeed; }
    void setSpeed(float speed) { m_speed.data = speed; }
};

#endif //NEUVISYS_MOTOR_H
