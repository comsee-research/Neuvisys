//
// Created by thomas on 03/03/2021.
//

#ifndef NEUVISYS_NEUVISYS_HPP
#define NEUVISYS_NEUVISYS_HPP

#include <chrono>
#include <random>
#include <utility>

#include "src/network/SpikingNetwork.hpp"
#include "src/dependencies/json.hpp"
#include "cnpy.h"

class Neuvisys {

public:
    Neuvisys(std::string networkPath, std::string events, size_t nbPass);
    void multiplePass();

protected:
    std::string m_networkPath;
    std::string m_events;
    size_t m_nbPass;
    long m_iterations;
    cv::Mat m_leftEventDisplay;
    cv::Mat m_rightEventDisplay;
    std::map<size_t, cv::Mat> m_weightDisplay;
    std::map<size_t, std::vector<long>> m_spikeTrain;
    std::chrono::time_point<std::chrono::system_clock> m_frameTime;
    std::chrono::time_point<std::chrono::system_clock> m_trackingTime;

    size_t m_layer = 0;
    size_t m_camera = 0;
    size_t m_layer2 = 0;
    size_t m_synapse = 0;
    size_t m_idSimple = 0;
    size_t m_idComplex = 0;

    size_t m_precisionEvent = 30000; // µs
    size_t m_rangePotential = 10000; // µs
    size_t m_precisionPotential = 10000; // µs
    size_t m_rangeSpiketrain = 1000000; // µs
    size_t m_precisionSpiketrain = 30000; // µs

private:
    void main_loop(SpikingNetwork &spinet);
    void runSpikingNetwork(SpikingNetwork &spinet, Event &event, size_t sizeArray);
};

#endif //NEUVISYS_NEUVISYS_HPP
