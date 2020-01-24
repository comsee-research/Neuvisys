#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#include "src/Neurons/DelayedSpikingNeuron.hpp"
#include "src/Neurons/DelayedSpikingNeuron2.hpp"
#include "src/Neurons/OrientedSpikingNeuron.hpp"
#include <vector>
#include <array>

class SpikingNetwork {
    std::vector<DelayedSpikingNeuron> m_neurons;
    std::vector<std::vector<size_t>> m_retina;
    void generateNeuronConfiguration();
    void assignNeurons();
public:
    SpikingNetwork();
    void addEvent(long timestamp, int x, int y, bool polarity);
    void updateNeurons(long time);
    void updateDisplay(long time, std::vector<cv::Mat> &displays);
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
