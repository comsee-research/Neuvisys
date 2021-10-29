//
// Created by alphat on 06/05/2021.
//

#ifndef NEUVISYS_DV_NETWORK_HANDLE_HPP
#define NEUVISYS_DV_NETWORK_HANDLE_HPP

#include "SpikingNetwork.hpp"

std::vector<Event> mono(const std::string &events, size_t nbPass);
std::vector<Event> stereo(const std::string &events, size_t nbPass);

class NetworkHandle {
    SpikingNetwork m_spinet;
    std::map<std::string, std::vector<double>> m_saveData;
    double m_reward{};

    NetworkConfig m_conf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;
    NeuronConfig m_criticNeuronConf;
    NeuronConfig m_actorNeuronConf;

public:
    explicit NetworkHandle(const std::string &networkPath);

    void multiplePass(const std::string &events, size_t nbPass);
    void updateActor(long timestamp, size_t actor);
    double storeLearningMetrics(double time, size_t nbEvents);
    void transmitReward(double reward);
    void transmitEvents(const std::vector<Event> &eventPacket);
    std::vector<uint64_t> resolveMotor();
    void learningDecay(size_t iteration);
    void save(size_t nbRun, const std::string &eventFileName);
    void trackNeuron(long time, size_t id = 0, size_t layer = 0);

    double getScore(long time);
    std::reference_wrapper<Neuron> &getNeuron(size_t index, size_t layer);
    std::map<std::string, std::vector<double>> &getSaveData() { return m_saveData; }
    uint64_t getLayout(size_t layer, Position pos) { return m_spinet.getLayout()[layer][{ pos.x(), pos.y(), pos.z() }]; }
    cv::Mat getWeightNeuron(size_t idNeuron, size_t layer, size_t camera, size_t synapse, size_t z);
    cv::Mat getSummedWeightNeuron(size_t idNeuron, size_t layer);
    NetworkConfig getNetworkConfig() { return m_conf; }
    NeuronConfig getSimpleNeuronConfig() { return m_simpleNeuronConf; }
    NeuronConfig getComplexNeuronConfig() { return m_complexNeuronConf; }
    NeuronConfig getCriticNeuronConfig() { return m_criticNeuronConf; }
    NeuronConfig getActorNeuronConfig() { return m_actorNeuronConf; }
    std::vector<size_t> getNetworkStructure();
};

#endif //NEUVISYS_DV_NETWORK_HANDLE_HPP
