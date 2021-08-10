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
    NetworkConfig m_conf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;
    NeuronConfig m_motorNeuronConf;
    long m_iterations{};

    double m_reward{};
    double m_bias{};
    size_t m_rewadIter{};
    std::vector<double> m_listReward;

    std::vector<Eigen::Tensor<double, SIMPLEDIM>> m_sharedWeightsSimple;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsComplex;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsMotor;

    std::vector<std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t>> m_layout;
    std::vector<std::vector<std::reference_wrapper<Neuron>>> m_neurons;

    std::vector<SimpleNeuron> m_simpleNeurons;
    std::vector<ComplexNeuron> m_complexNeurons;
    std::vector<MotorNeuron> m_motorNeurons;
    std::vector<std::vector<uint64_t>> m_pixelMapping;

public:
    explicit SpikingNetwork(const std::string &conf);
    void runEvents(const std::vector<Event> &eventPacket, double reward);
    void runEvent(const Event &event);
    void addEvent(const Event &event);
    void updateNeuronsParameters(long time);
    void trackNeuron(long time, size_t id = 0, size_t layer = 0);
    void loadNetwork();
    void saveNetwork(size_t nbRun, const std::string& eventFileName);
    void updateReward(double reward);
    std::vector<uint64_t> resolveMotor();

    Neuron &getNeuron(size_t index, size_t layer);
    std::vector<size_t> getNetworkStructure();
    NetworkConfig getNetworkConfig() { return m_conf; }
    NeuronConfig getSimpleNeuronConfig() { return m_simpleNeuronConf; }
    NeuronConfig getComplexNeuronConfig() { return m_complexNeuronConf; }
    uint64_t getLayout(size_t layer, uint64_t x, uint64_t y, uint64_t z) { return m_layout[layer][{x, y, z}]; }
    std::vector<double> &getRewards() { return m_listReward; }
    cv::Mat getWeightNeuron(size_t idNeuron, size_t layer, size_t camera, size_t synapse, size_t z);
    [[nodiscard]] double getBias() const { return m_bias; };

private:
    void updateNeurons(long time);
    void saveNeuronsStates();
    void loadWeights();
    void generateWeightSharing(const std::string &neuronType, const std::vector<size_t> &neuronSizes);
    Position findPixelComplexNeuron(ComplexNeuron &neuron);
    void addLayer(const std::string &neuronType, const std::string &sharingType, const std::vector<std::vector<size_t>> &layerPatches,
                  const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes);
    void connectLayer(bool inhibition, const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes);
    void addNeuronEvent(const Neuron &neuron);
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
