//
// Created by Thomas on 06/05/2021.
//

#ifndef NEUVISYS_DV_NETWORK_HANDLE_HPP
#define NEUVISYS_DV_NETWORK_HANDLE_HPP

#include "SpikingNetwork.hpp"

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
    hsize_t countPass = 0;
    uint64_t firstTimestamp = 0;
    uint64_t lastTimestamp = 0;
};

struct SaveTime {
    double action{};
    double update{};
    double console{};
    double display{};

    explicit SaveTime(double initTime) : action(initTime), update(initTime), console(initTime), display(initTime) {};
};

/**
 * Used as an abstraction layer on top of the SpikingNetwork class.
 * It offers functions used for communication between the environment (mainly the incoming flow of events) and the spiking neural network.
 * Example:
 *      network = NetworkHandle("/path/to/network/");
 *      network.transmitEvents(eventPacket);
 */
class NetworkHandle {
    SpikingNetwork m_spinet;
    ReinforcementLearningConfig m_rlConf;
    NetworkConfig m_networkConf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;
    NeuronConfig m_criticNeuronConf;
    NeuronConfig m_actorNeuronConf;
    H5EventFile m_eventFile;
    SaveTime m_saveTime;

    std::map<std::string, std::vector<double>> m_saveData;
    double m_reward{};
    double m_error{};
    double m_neuromodulator{};
    std::string m_eventsPath;
    size_t m_nbEvents{};
    int m_action = 0;
    bool m_exploration{};
    size_t m_iteration{};
    size_t m_criticCount{};
    size_t m_actionCount{};
    size_t m_scoreCount{};
    size_t m_countEvents{};
    size_t m_totalNbEvents{};
    size_t m_saveCount{};
    double m_endTime{};
    double m_averageEventRate{};
    double m_decayCritic = 1.0;
    double m_decayActor = 1.0;

public:
    NetworkHandle();

    explicit NetworkHandle(const std::string& eventsPath, double time);

    explicit NetworkHandle(const std::string &networkPath);

    explicit NetworkHandle(const std::string &networkPath, const std::string &eventsPath);

    void setEventPath(const std::string &eventsPath);

    bool loadEvents(std::vector<Event> &events, size_t nbPass);

    void feedEvents(const std::vector<Event> &events);

    void computeCriticNeuromodulator();

    void computeActorNeuromodulator();

    void updateActor();

    void saveValueMetrics(long time, size_t nbEvents);

    void saveActionMetrics();

    void transmitReward(double reward);

    void transmitEvent(const Event &event);

    std::vector<uint64_t> resolveMotor();

    void learningDecay(double time);

    void save(const std::string &eventFileName, size_t nbRun);

    void saveStatistics(size_t simulation, size_t sequence, bool reset=false);

    void trackNeuron(long time, size_t id = 0, size_t layer = 0);

    double valueFunction(long time);

    double valueDerivative(const std::vector<double> &value);

    void actionSelection(const std::vector<uint64_t> &actionsActivations, double explorationFactor);

    void updateNeurons(size_t time);

    int learningLoop(long lastTimestamp, double time, size_t nbEvents, std::string &msg);

    double getScore(long nbPreviousReward);

    std::map<std::string, std::vector<double>> &getSaveData() { return m_saveData; }

    std::reference_wrapper<Neuron> &getNeuron(size_t index, size_t layer) { return m_spinet.getNeuron(index, layer); }

    const std::vector<size_t> &getNetworkStructure() { return m_spinet.getNetworkStructure(); }

    uint64_t getLayout(size_t layer, Position pos) { return m_spinet.getLayout()[layer][{pos.x(), pos.y(), pos.z()}]; }

    cv::Mat neuronWeightMatrix(size_t idNeuron, size_t layer, size_t camera, size_t synapse, size_t z);

    cv::Mat getSummedWeightNeuron(size_t idNeuron, size_t layer);

    NetworkConfig getNetworkConfig() { return m_networkConf; }

    ReinforcementLearningConfig getRLConfig() { return m_rlConf; }

    NeuronConfig getSimpleNeuronConfig() { return m_simpleNeuronConf; }

    NeuronConfig getComplexNeuronConfig() { return m_complexNeuronConf; }

    NeuronConfig getCriticNeuronConfig() { return m_criticNeuronConf; }

    NeuronConfig getActorNeuronConfig() { return m_actorNeuronConf; }

    [[nodiscard]] uint64_t getFirstTimestamp() const { return m_eventFile.firstTimestamp; }

    [[nodiscard]] uint64_t getLastTimestamp() const { return m_eventFile.lastTimestamp; }

    void changeNeuronToTrack(int n_x, int n_y);

    void lateralRandom();

    void inhibitionShuffle(int case_);

    void assignOrientation(int z, int ori);

    void assignComplexOrientation(int neur, int ori);

private:
    void load();

    void loadNpzEvents(std::vector<Event> &events, size_t nbPass = 1);

    void openH5File();

    bool loadHDF5Events(std::vector<Event> &events, size_t nbPass);

    void updateCritic();

    void readFirstAndLastTimestamp();

    void resetAllNeurons();
};

#endif //NEUVISYS_DV_NETWORK_HANDLE_HPP
