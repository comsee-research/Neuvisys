//
// Created by Thomas on 14/04/2021.
//

#ifndef NEUVISYSTHREADSIMULATION_H
#define NEUVISYSTHREADSIMULATION_H

#include "../Neuvisysthread.h"
#include "../../simulator/SimulationInterface.hpp"

class NeuvisysThreadSimulation : public NeuvisysThread {
Q_OBJECT

public:
    NeuvisysThreadSimulation(int argc, char **argv, QObject *parent = nullptr);

    ~NeuvisysThreadSimulation() override;

    bool init();

    void run() override;

private:

    void readEventsSimulation();

    void launchSimulation(NetworkHandle &network);
};

#endif // NEUVISYSTHREADSIMULATION_H
