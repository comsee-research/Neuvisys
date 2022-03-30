//
// Created by alphat on 06/05/2021.
//

#ifndef NEUVISYS_DV_NETWORK_HANDLE_HPP
#define NEUVISYS_DV_NETWORK_HANDLE_HPP

#include "SpikingNetwork.hpp"
#include "H5Cpp.h"
#include "hdf5.h"
#include "blosc_filter.h"

/*
 * Used as an abstraction layer on top of the SpikingNetwork class.
 * It offers functions used for communication between the environment (mainly the incoming flow of events) and the spiking neural network.
 * Example:
 *      network = NetworkHandle("/path/to/network/");
 *      network.transmitEvents(eventPacket);
 */
class NetworkHandle {
    SpikingNetwork m_spinet;
    std::map<std::string, std::vector<double>> m_saveData;
    double m_reward{};

    NetworkConfig m_networkConf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;
    NeuronConfig m_criticNeuronConf;
    NeuronConfig m_actorNeuronConf;

    double m_actionTime{}, m_updateTime{}, m_consoleTime{};
    int m_action{};
    size_t m_iteration{};
    size_t m_countEvents{};
    size_t saveCount{};
public:
    NetworkHandle(const std::string &networkPath, double time);

    std::vector<Event> loadEvents(const std::string &events, size_t nbPass);

    void feedEvents(const std::vector<Event> &events);

    void updateActor(long time, size_t actor);

    void saveValueMetrics(long time, size_t nbEvents);

    void saveActionMetrics(size_t action, bool exploration);

    void transmitReward(double reward);

    void transmitEvent(const Event &event);

    std::vector<uint64_t> resolveMotor();

    void learningDecay(size_t iteration);

    void save(const std::string &eventFileName, size_t nbRun);

    void trackNeuron(long time, size_t id = 0, size_t layer = 0);

    static std::vector<Event> mono(const std::string &events, size_t nbPass);

    static std::vector<Event> stereo(const std::string &events, size_t nbPass);

    double valueFunction(long time);

    double valueDerivative(const std::vector<double> &value);

    std::pair<int, bool> actionSelection(const std::vector<uint64_t> &actionsActivations, double explorationFactor);

    int learningLoop(long lastTimestamp, double time, size_t nbEvents, std::string &msg);

    double getScore(long time);

    std::map<std::string, std::vector<double>> &getSaveData() { return m_saveData; }

    std::reference_wrapper<Neuron> &getNeuron(size_t index, size_t layer) { return m_spinet.getNeuron(index, layer); }

    const std::vector<size_t> &getNetworkStructure() { return m_spinet.getNetworkStructure(); }

    uint64_t getLayout(size_t layer, Position pos) { return m_spinet.getLayout()[layer][{pos.x(), pos.y(), pos.z()}]; }

    cv::Mat getWeightNeuron(size_t idNeuron, size_t layer, size_t camera, size_t synapse, size_t z);

    cv::Mat getSummedWeightNeuron(size_t idNeuron, size_t layer);

    NetworkConfig getNetworkConfig() { return m_networkConf; }

    NeuronConfig getSimpleNeuronConfig() { return m_simpleNeuronConf; }

    NeuronConfig getComplexNeuronConfig() { return m_complexNeuronConf; }

    NeuronConfig getCriticNeuronConfig() { return m_criticNeuronConf; }

    NeuronConfig getActorNeuronConfig() { return m_actorNeuronConf; }

private:
    void load();
};

#endif //NEUVISYS_DV_NETWORK_HANDLE_HPP
