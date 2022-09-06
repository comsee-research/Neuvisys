//
// Created by thomas on 05/09/2022.
//

#ifndef NEUVISYS_LAUNCHER_HPP
#define NEUVISYS_LAUNCHER_HPP

#include <simulator/SimulationInterface.hpp>
#include <network/NetworkHandle.hpp>

void launchNetworkSimulation(std::string &networkPath, bool actions, double simTime);

int launchSimulationWithoutNetwork(double simTime);

void launchValidationMultiWeights();

#endif //NEUVISYS_LAUNCHER_HPP
