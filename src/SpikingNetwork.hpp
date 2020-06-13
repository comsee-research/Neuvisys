#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#include <vector>
#include <array>
#include <src/neurons/PoolingNeuron.hpp>
#include "src/neurons/SpatioTemporalNeuron.hpp"
#include "src/dependencies/gnuplot.h"

class SpikingNetwork {
    NetworkConfig &conf;
    NeuronConfig m_neuronConf;
    NeuronConfig m_poolingNeuronConf;

    std::vector<xt::xarray<double>> m_sharedWeights;
    std::vector<xt::xarray<double>> m_sharedWeightsPooling;

    std::vector<SpatioTemporalNeuron> m_neurons;
    std::vector<PoolingNeuron> m_poolingNeurons;
    std::vector<std::vector<size_t>> m_retina;
    std::vector<std::vector<size_t>> m_poolingRetina;

    std::deque<double> m_potentials;
    std::deque<long> m_timestamps;
    std::vector<size_t> m_spikes;

    Luts m_luts;
    GnuplotPipe gp = GnuplotPipe(false);

    void generateNeuronConfiguration();
    void assignNeurons();

    void potentialDisplay();
    void weightDisplay(cv::Mat &display);
    void spikingDisplay(cv::Mat &display);
    void multiPotentialDisplay(long time, cv::Mat &display);
public:
    explicit SpikingNetwork(NetworkConfig &conf);
    void addEvent(long timestamp, int x, int y, bool polarity);
    void updateNeurons(long time);
    void updateDisplay(long time, std::vector<cv::Mat> &displays);
    void updateNeuronsParameters();
    void saveWeights();
    void loadWeights();
    SpatioTemporalNeuron getNeuron(unsigned long index);
private:
    void simpleConfiguration(const std::vector<long>& delays);
    void weightSharingConfiguration(const std::vector<long>& delays);
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
