#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#include <vector>
#include <array>
#include <src/network/neurons/ComplexNeuron.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include "src/network/neurons/SimpleNeuron.hpp"

class SpikingNetwork {
    NetworkConfig &m_conf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;

    std::vector<Eigen::Tensor<double, SIMPLEDIM>> m_sharedWeightsSimple;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsComplex;

    std::vector<SimpleNeuron> m_simpleNeurons;
    std::vector<ComplexNeuron> m_complexNeurons;
    std::vector<std::vector<uint64_t>> m_retina;

    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t> m_layout1;
    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t> m_layout2;

    uint64_t m_nbSimpleNeurons;
    uint64_t m_nbComplexNeurons;

    Luts m_simpleluts;
    Luts m_complexluts;
public:
    explicit SpikingNetwork(NetworkConfig &conf);

    ~SpikingNetwork();
    void addEvent(Event event);
    void addComplexEvent(SimpleNeuron &neuron);
    void updateNeurons(long time);
    void updateNeuronsParameters(long time);
    void saveNeuronsStates();
    void loadWeights(bool simpleNeuronStored, bool complexNeuronStored);

    SimpleNeuron getNeuron(unsigned long index) { return m_simpleNeurons[index]; }
    [[nodiscard]] size_t getNumberNeurons() const { return m_nbSimpleNeurons; }
    [[nodiscard]] size_t getNumberPoolingNeurons() const { return m_nbComplexNeurons; }
    NetworkConfig getNetworkConfig() { return m_conf; }
    NeuronConfig getSimpleNeuronConfig() { return m_simpleNeuronConf; }
    NeuronConfig getComplexNeuronConfig() { return m_complexNeuronConf; }
    uint64_t getLayout1(uint64_t x, uint64_t y, uint64_t z) { return m_layout1[{x, y, z}]; }
    uint64_t getLayout2(uint64_t x, uint64_t y, uint64_t z) { return m_layout2[{x, y, z}]; }
    void trackNeuron(const long time, const size_t simpleId = 0, const size_t complexId = 0);

    cv::Mat getWeightNeuron(size_t idNeuron, size_t camera, size_t synapse, size_t neuronType, size_t layer);
    const std::vector<long> &getSpikingNeuron(size_t idNeuron, size_t neuronType);
    const std::vector<std::pair<double, long>> &getPotentialNeuron(size_t idNeuron, size_t neuronType);

private:
    bool simpleNeuronsFilesExists() const;
    bool complexNeuronsFilesExists() const;
    void generateWeightSharing(bool simpleNeuronStored, bool complexNeuronStored);
    void generateNeuronConfiguration();
    void assignNeurons();
    Position findPixelComplexNeuron(ComplexNeuron &neuron);

};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
