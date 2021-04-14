//
// Created by alphat on 14/04/2021.
//

#ifndef NEUVISYS_NETWORKHANDLE_HPP
#define NEUVISYS_NETWORKHANDLE_HPP

#include <chrono>
#include <random>

#include "src/dependencies/json.hpp"
#include "cnpy.h"
#include "src/network/SpikingNetwork.hpp"

class NetworkHandle {

public:
    NetworkHandle(std::string networkPath, std::string events, size_t nbPass);
    void multiplePass();

protected:
    std::string m_networkPath;
    std::string m_events;
    size_t m_nbPass;
    long m_iterations;
    std::chrono::time_point<std::chrono::system_clock> m_frameTime;
    std::chrono::time_point<std::chrono::system_clock> m_trackingTime;

    size_t m_precisionEvent = 30000; // µs
    size_t m_precisionPotential = 10000; // µs

private:
    void main_loop(SpikingNetwork &spinet);
    void runSpikingNetwork(SpikingNetwork &spinet, Event &event, size_t sizeArray);
};


#endif //NEUVISYS_NETWORKHANDLE_HPP
