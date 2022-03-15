//
// Created by thomas on 19/11/2021.
//

#ifndef NEUVISYS_BRUSHLESSMOTOR_HPP
#define NEUVISYS_BRUSHLESSMOTOR_HPP

#include "libserie/Faulhaber.hpp"
#include "../network/Util.hpp"

class BrushlessMotor {
protected:
    Faulhaber m_motor;
    double m_pos{};
    double m_speed{};
    double m_jitterPos{};
    double m_jitterSpeed{};
    double m_lowerBound{};
    double m_upperBound{};

public:
    BrushlessMotor(size_t motorAdress, const std::string &port);
    ~BrushlessMotor();
    void setSpeed(int speed);
    void setPosition(int position);
    double getPosition();
    void setBounds(double lower, double upper) { m_lowerBound = lower; m_upperBound = upper; }
    [[nodiscard]] bool isActionValid(double position, double projection) const;
    void jitterPos(double dt);

    void jitterSpeed(double dt);
};

#endif //NEUVISYS_BRUSHLESSMOTOR_HPP
