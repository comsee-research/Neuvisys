//
// Created by alphat on 06/05/2021.
//

#ifndef NEUVISYS_DV_NETWORK_HANDLE_HPP
#define NEUVISYS_DV_NETWORK_HANDLE_HPP

#include "SpikingNetwork.hpp"
#include "H5Cpp.h"
#include "hdf5.h"

struct H5EventFile {
    H5::H5File file;
    H5::Group group;
    H5::DataSet timestamps;
    H5::DataSet x;
    H5::DataSet y;
    H5::DataSet polarities;
    H5::DataSet cameras;
    hsize_t dims;
    hsize_t packetSize = 10000;
    hsize_t offset = 0;
};

struct SaveTime {
    double action{};
    double update{};
    double console{};
    double display{};

    explicit SaveTime(double initTime) : action(initTime), update(initTime), console(initTime), display(initTime) {};
};

/*
 * Used as an abstraction layer on top of the SpikingNetwork class.
 * It offers functions used for communication between the environment (mainly the incoming flow of events) and the spiking neural network.
 * Example:
 *      network = NetworkHandle("/path/to/network/");
 *      network.transmitEvents(eventPacket);
 */
class NetworkHandle {
    SpikingNetwork m_spinet;
    NetworkConfig m_networkConf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;
    NeuronConfig m_criticNeuronConf;
    NeuronConfig m_actorNeuronConf;
    H5EventFile m_eventFile;
    SaveTime m_saveTime;

    std::map<std::string, std::vector<double>> m_saveData;
    double m_reward{};
    std::string m_eventsPath;
    size_t m_nbEvents{};
    int m_action{};
    size_t m_iteration{};
    size_t m_scoreIteration{};
    size_t m_countEvents{};
    size_t m_saveCount{};
    double m_timeStep{};

public:
    NetworkHandle();

    explicit NetworkHandle(std::string eventsPath);

    NetworkHandle(const std::string &networkPath, double time);

    NetworkHandle(const std::string &networkPath, double time, const std::string &eventsPath);

    bool loadEvents(std::vector<Event> &events, size_t nbPass);

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

    void loadNpzEvents(std::vector<Event> &events, size_t nbPass = 1);

    double valueFunction(long time);

    double valueDerivative(const std::vector<double> &value);

    std::pair<int, bool> actionSelection(const std::vector<uint64_t> &actionsActivations, double explorationFactor);

    int learningLoop(long lastTimestamp, double time, size_t nbEvents, std::string &msg);

    double getScore(long nbPreviousReward);

    void setTimeStep(double timeStep) { m_timeStep = timeStep; std::cout << m_timeStep << std::endl; }

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

    void loadH5File();

    bool loadHDF5Events(std::vector<Event> &events);

    bool loadHDF5EventsStereo(std::vector<Event> &events);
};

#endif //NEUVISYS_DV_NETWORK_HANDLE_HPP
