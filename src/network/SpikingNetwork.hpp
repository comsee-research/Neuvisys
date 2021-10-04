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
    long m_iterations{};

    std::vector<std::pair<uint64_t, float>> motorMapping;

    double m_reward{};
    double m_averageReward{};
    size_t m_rewardIter{};
    std::vector<double> m_listReward;
    std::vector<double> m_listValue;
    std::vector<double> m_listValueDot;
    std::vector<double> m_listTDError;
    std::vector<double> m_listTDActions;
    std::vector<size_t> m_listEvents;

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

public:
    explicit SpikingNetwork(const std::string &conf);
    void addLayer(const std::string &neuronType, const std::string &sharingType, bool inhibition, const std::vector<std::vector<size_t>>& layerPatches,
                  const std::vector<size_t>& layerSizes, const std::vector<size_t>& neuronSizes, const std::vector<size_t> &neuronOverlap, size_t layerToConnect);
    void runEvents(const std::vector<Event> &eventPacket, double reward);
    void runEvent(const Event &event);
    void addEvent(const Event &event);
    void updateNeuronsParameters(long time);
    void trackNeuron(long time, size_t id = 0, size_t layer = 0);
    void loadNetwork();
    void saveNetwork(size_t nbRun, const std::string& eventFileName);
    void transmitReward(double reward, size_t nbEvents);
    std::vector<uint64_t> resolveMotor();
    double updateTDError(double time, bool store = false);

    std::reference_wrapper<Neuron> &getNeuron(size_t index, size_t layer);
    std::vector<size_t> getNetworkStructure();
    NetworkConfig getNetworkConfig() { return m_conf; };
    NeuronConfig getSimpleNeuronConfig() { return m_simpleNeuronConf; }
    NeuronConfig getComplexNeuronConfig() { return m_complexNeuronConf; }
    uint64_t getLayout(size_t layer, Position pos) { return m_layout[layer][{ pos.x(), pos.y(), pos.z() }]; }
    cv::Mat getWeightNeuron(size_t idNeuron, size_t layer, size_t camera, size_t synapse, size_t z);
    [[nodiscard]] double getAverageReward() const { return m_averageReward; }
    std::vector<double> &getRewards() { return m_listReward; }
    std::vector<double> &getListValue() { return m_listValue; }
    std::vector<double> &getListValueDot() { return m_listValueDot; }
    std::vector<double> &getListTDError() { return m_listTDError; }

private:
    void updateNeurons(long time);
    void saveNeuronsStates();
    void loadWeights();
    void generateWeightSharing(const std::string &neuronType, const std::vector<size_t> &neuronSizes, size_t nbNeurons);
    void addNeuronEvent(const Neuron &neuron);
    void connectLayer(bool inhibition, size_t layerToConnect, const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes);
};

#endif //NEUVISYS_DV_SPIKING_NETWORK_HPP
