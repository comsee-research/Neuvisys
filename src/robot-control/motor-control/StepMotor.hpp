//
// Created by thomas on 19/11/2021.
//

#ifndef NEUVISYS_STEPMOTOR_HPP
#define NEUVISYS_STEPMOTOR_HPP

#include "libserie/Faulhaber.hpp"
#include "../../network/Utils.hpp"

class StepMotor {
protected:
    Faulhaber m_motor;
    double m_pos{};
    double m_speed{};
    double m_jitterPos{};
    double m_jitterSpeed{};
    double m_lowerBound{};
    double m_upperBound{};

public:
    StepMotor(size_t motorAdress, const std::string &port);
    ~StepMotor();
    void setSpeed(int speed);
    void setPosition(int position);
    double getPosition();
    void setBounds(double lower, double upper) { m_lowerBound = lower; m_upperBound = upper; }
    [[nodiscard]] bool isActionValid(double position, double projection) const;
    void jitterPos(double dt);

    void jitterSpeed(double dt);
};

#endif //NEUVISYS_STEPMOTOR_HPP
