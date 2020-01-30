#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#include "src/Neurons/DelayedSpikingNeuron.hpp"
#include "src/Neurons/DelayedSpikingNeuron2.hpp"
#include "src/Neurons/OrientedSpikingNeuron.hpp"
#include <vector>
#include <array>
#include "gnuplot.h"

class SpikingNetwork {
    std::vector<OrientedSpikingNeuron> m_neurons;
    std::vector<std::vector<size_t>> m_retina;

    std::vector<bool> m_firings;
    std::deque<double> m_potentials;
    std::deque<long> m_timestamps;
    GnuplotPipe gp;

    void generateNeuronConfiguration();
    void assignNeurons();
public:
    SpikingNetwork();
    void addEvent(long timestamp, int x, int y, bool polarity);
    void updateNeurons(long time);
    void updateDisplay(long time, std::vector<cv::Mat> &displays);
    void displayInformations();
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
