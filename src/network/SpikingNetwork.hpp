#ifndef NEUVISYS_DV_SPIKING_NETWORK_HPP
#define NEUVISYS_DV_SPIKING_NETWORK_HPP

#include <utility>
#include <vector>
#include <array>
#include "neurons/SimpleNeuron.hpp"
#include "neurons/ComplexNeuron.hpp"
#include "neurons/MotorNeuron.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

class SpikingNetwork {
    NetworkConfig &m_conf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;
    NeuronConfig m_motorNeuronConf;
    long m_iterations{};
    std::chrono::time_point<std::chrono::system_clock> m_frameTime;
    std::chrono::time_point<std::chrono::system_clock> m_trackingTime;

    size_t m_precisionEvent = 30000; // µs
    size_t m_precisionPotential = 10000; // µs

    double m_reward;

    std::vector<Eigen::Tensor<double, SIMPLEDIM>> m_sharedWeightsSimple;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsComplex;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsMotor;

    std::vector<SimpleNeuron> m_simpleNeurons;
    std::vector<ComplexNeuron> m_complexNeurons;
    std::vector<MotorNeuron> m_motorNeurons;
    std::vector<std::vector<uint64_t>> m_pixelMapping;
    std::vector<bool> m_motorActivations;

    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t> m_layout1;
    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t> m_layout2;
    std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t> m_layout3;

    uint64_t m_nbSimpleNeurons;
    uint64_t m_nbComplexNeurons;
    uint64_t m_nbMotorNeurons;

//    Luts m_simpleluts;
//    Luts m_complexluts;
public:
    SpikingNetwork(NetworkConfig &conf);
    ~SpikingNetwork();
    std::vector<bool> run(const std::vector<Event>& eventPacket, const double reward);

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
    void addEvent(Event event);
    void addComplexEvent(SimpleNeuron &neuron);
    void addMotorEvent(ComplexNeuron &neuron);
    void updateNeurons(long time);
    void updateNeuronsParameters(long time);
    void saveNetworkLearningTrace(size_t nbRun, const std::string& eventFileName);
    void saveNeuronsStates();
    bool simpleNeuronsFilesExists() const;
    bool complexNeuronsFilesExists() const;
    bool motorNeuronsFilesExists() const;
    void loadWeights(bool simpleNeuronStored, bool complexNeuronStored, bool motorNeuronStored);
    void generateWeightSharing(bool simpleNeuronStored, bool complexNeuronStored, bool motorNeuronStored);
    void generateNeuronConfiguration();
    void assignNeurons();
    Position findPixelComplexNeuron(ComplexNeuron &neuron);
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
