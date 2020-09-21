#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#include <vector>
#include <array>
#include <src/neurons/PoolingNeuron.hpp>
#include "src/neurons/SpatioTemporalNeuron.hpp"
#include "src/dependencies/gnuplot.h"

class SpikingNetwork {
    NetworkConfig &conf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;

    std::vector<xt::xarray<double>> m_sharedWeightsSimple;
    std::vector<xt::xarray<double>> m_sharedWeightsComplex;

    std::vector<SpatioTemporalNeuron> m_simpleNeurons;
    std::vector<PoolingNeuron> m_complexNeurons;
    std::vector<std::vector<size_t>> m_simpleRetina;
    std::vector<std::vector<size_t>> m_complexRetina;

    std::vector<Position> m_layout1;
    std::vector<Position> m_layout2;

    std::deque<double> m_potentials;
    std::deque<long> m_timestamps;
    std::vector<size_t> m_simpleSpikes;
    std::vector<size_t> m_complexSpikes;

    size_t m_nbSimpleNeurons;
    size_t m_nbComplexNeurons;

    Luts m_simpleluts;
    Luts m_complexluts;
    GnuplotPipe gp = GnuplotPipe(false);
public:
    explicit SpikingNetwork(NetworkConfig &conf);

    ~SpikingNetwork();
    void addEvent(long timestamp, int x, int y, bool polarity);
    void updateNeurons(long time);
    void updateDisplay(long time, std::map<std::string, cv::Mat> &displays);
    void updateNeuronsParameters(long time);
    void saveWeights();
    void loadWeights();
    SpatioTemporalNeuron getNeuron(unsigned long index);

    [[nodiscard]] size_t getNumberNeurons() const {return m_nbSimpleNeurons;}
    [[nodiscard]] size_t getNumberPoolingNeurons() const {return m_nbComplexNeurons;}
    void trackNeuron(long time);
private:
    void generateWeightSharing();
    void generateNeuronConfiguration();
    void assignNeurons();

    [[maybe_unused]] void potentialDisplay();
    void weightDisplay(cv::Mat &display);
    void weight2Display(cv::Mat &display);
    void spikingDisplay(cv::Mat &display);
    void spiking2Display(cv::Mat &display);
    void multiPotentialDisplay(long time, cv::Mat &display);
    void multiPotential2Display(long time, cv::Mat &display);
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
