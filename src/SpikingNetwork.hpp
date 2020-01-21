#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP

#include "SpikingNeuron.hpp"
#include <vector>
#include <array>

class SpikingNetwork {
    std::vector<SpikingNeuron> m_neurons;
    std::array<std::vector<size_t>, WIDTH*HEIGHT> m_retina;
    void generateNeuronConfiguration();
    void assignNeurons();
public:
    SpikingNetwork();
    int getNumberOfNeurons();
    SpikingNeuron getNeuron(size_t ind);
    void addEvent(long timestamp, int x, int y, bool polarity);
    void updateNeurons(long time);
    void updateNeuronsInformation(long time, std::vector<cv::Mat> &displays);
};
