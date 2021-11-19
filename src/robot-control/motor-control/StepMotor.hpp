//
// Created by thomas on 19/11/2021.
//

#ifndef NEUVISYS_STEPMOTOR_HPP
#define NEUVISYS_STEPMOTOR_HPP

#include "libserie/Faulhaber.hpp"

class StepMotor {
protected:
    Faulhaber m_motor;

public:
    StepMotor(const std::string &topic, size_t motorAdress, const std::string &port);
    ~StepMotor();
    void setSpeed(int speed);
    void setPosition(int position);
    double getPosition();
};

#endif //NEUVISYS_STEPMOTOR_HPP
