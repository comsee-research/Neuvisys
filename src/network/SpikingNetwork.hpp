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
    NeuronConfig m_criticNeuronConf;
    NeuronConfig m_actorNeuronConf;

    std::vector<Eigen::Tensor<double, SIMPLEDIM>> m_sharedWeightsSimple;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsComplex;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsCritic;
    std::vector<Eigen::Tensor<double, COMPLEXDIM>> m_sharedWeightsActor;

    std::vector<std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t>> m_layout;
    std::vector<std::vector<std::reference_wrapper<Neuron>>> m_neurons;

    std::vector<SimpleNeuron> m_simpleNeurons;
    std::vector<ComplexNeuron> m_complexNeurons;
    std::vector<MotorNeuron> m_criticNeurons;
    std::vector<MotorNeuron> m_actorNeurons;
    std::vector<std::vector<uint64_t>> m_pixelMapping;

    long m_iterations{};
    double m_neuromodulator{};

public:
    explicit SpikingNetwork(const std::string &conf);
    void addLayer(const std::string &neuronType, const std::string &sharingType, bool inhibition, const std::vector<std::vector<size_t>>& layerPatches,
                  const std::vector<size_t>& layerSizes, const std::vector<size_t>& neuronSizes, const std::vector<size_t> &neuronOverlap, size_t layerToConnect);
    void runEvent(const Event &event);
    void addEvent(const Event &event);
    void updateNeuronsParameters(long time);
    void loadWeights();
    void saveNetwork();
    double computeNeuromodulator(double time);
    void transmitNeuromodulator(double neuromodulator);
    void normalizeActions();

    std::vector<std::vector<std::reference_wrapper<Neuron>>> &getNeurons() { return m_neurons; }
    std::vector<std::map<std::tuple<uint64_t, uint64_t, uint64_t>, uint64_t>> &getLayout() { return m_layout; }

private:
    void updateNeurons(long time);
    void saveNeuronsStates();
    void generateWeightSharing(const std::string &neuronType, const std::vector<size_t> &neuronSizes, size_t nbNeurons);
    void addNeuronEvent(const Neuron &neuron);
    void connectLayer(bool inhibition, size_t layerToConnect, const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes);
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
