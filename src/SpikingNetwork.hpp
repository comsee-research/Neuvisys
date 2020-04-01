#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#include <vector>
#include <array>

#include "src/neurons/TemporalNeuron.hpp"
#include "src/neurons/SpatioTemporalNeuron.hpp"
#include "src/neurons/OrientedNeuron.hpp"
#include "src/dependencies/gnuplot.h"

class SpikingNetwork {
    //std::vector<OrientedNeuron> m_neurons;
    std::vector<SpatioTemporalNeuron> m_neurons; //TODO
    std::vector<std::vector<size_t>> m_retina;

    std::deque<double> m_potentials;
    std::deque<long> m_timestamps;
    GnuplotPipe gp = GnuplotPipe(false);

    void generateNeuronConfiguration();
    void assignNeurons();

    void potentialDisplay();
    void weightDisplay(cv::Mat &display);
    void spikingDisplay(cv::Mat &display);
    void multiPotentialDisplay(long time, cv::Mat &display);
public:
    SpikingNetwork();
    void addEvent(long timestamp, int x, int y, bool polarity);
    void updateNeurons(long time);
    void updateDisplay(long time, std::vector<cv::Mat> &displays);
    void updateNeuronsParameters();
    void saveWeights();
    void loadWeights();
    OrientedNeuron getNeuron(unsigned long index);
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
