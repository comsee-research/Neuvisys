#ifndef FAULHABER__HPP___
#define FAULHABER__HPP___

#include "comserie.hpp"

class Faulhaber : public ComSerie {
public:
    Faulhaber(int Adresse_moteur = 0, string port = "/dev/ttyUSB0");

    ~Faulhaber();

    void StartDrive();

    void StopDrive();

    void GetPosition(string &position);

    void SetAbsolutePosition(int position);

    void SetRelativePosition(int position);

    void SetSpeed(int speed);

private:
    string Adresse_moteur_;
};

#endif
