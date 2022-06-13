//
// Created by Thomas on 14/04/2021.
//

#ifndef NEUVISYS_DV_CONFIG_HPP
#define NEUVISYS_DV_CONFIG_HPP

#include <iostream>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include "../dependencies/json.hpp"

#define E3 1000 // µs
#define E6 1000000 // µs

using json = nlohmann::json;

class NetworkConfig {
    /***** Display parameters *****/
    std::string m_networkPath;
    std::string m_networkConfigPath;

    /***** Spiking Neural Network layout parameters *****/
    size_t nbCameras{};
    size_t neuron1Synapses{};
    std::string sharingType{};
    size_t vfWidth{};
    size_t vfHeight{};
    double measurementInterval{};

    std::vector<std::string> layerCellTypes;
    std::vector<std::vector<std::string>> layerInhibitions;
    std::vector<int> interLayerConnections;
    std::vector<std::vector<std::vector<size_t>>> layerPatches;
    std::vector<std::vector<size_t>> layerSizes;
    std::vector<std::vector<size_t>> neuronSizes;
    std::vector<std::vector<size_t>> neuronOverlap;
    std::vector<int> neuronInhibitionRange;

public:
    NetworkConfig();

    explicit NetworkConfig(const std::string& configFile);

    void loadNetworkLayout();

    std::string &getNetworkPath() { return m_networkPath; }

    std::string &getNetworkConfigPath() { return m_networkConfigPath; }

    std::string &getSharingType() { return sharingType; }

    [[nodiscard]] size_t getNbCameras() const { return nbCameras; }

    [[nodiscard]] size_t getNeuron1Synapses() const { return neuron1Synapses; }

    [[nodiscard]] size_t getVfWidth() const { return vfWidth; }

    [[nodiscard]] size_t getVfHeight() const { return vfHeight; }

    [[nodiscard]] double getMeasurementInterval() const { return measurementInterval; }

    std::vector<std::string> &getLayerCellTypes() { return layerCellTypes; }

    std::vector<std::vector<std::string>> &getLayerInhibitions() { return layerInhibitions; }

    std::vector<int> &getInterLayerConnections() { return interLayerConnections; }

    std::vector<std::vector<std::vector<size_t>>> getLayerPatches() { return layerPatches; }

    std::vector<std::vector<size_t>> getLayerSizes() { return layerSizes; }

    std::vector<std::vector<size_t>> getNeuronSizes() { return neuronSizes; }

    std::vector<std::vector<size_t>> getNeuronOverlap() { return neuronOverlap; }

    std::vector<int> getNeuronInhibitionRange() { return neuronInhibitionRange; }

    static void createNetwork(const std::string &directory);
};

class NeuronConfig {
public:
    NeuronConfig();

    NeuronConfig(const std::string &configFile, size_t type);

/***** Neurons internal parameters *****/
    double TAU_M{}; // μs
    double TAU_LTP{}; // μs
    double TAU_LTD{}; // μs
    double TAU_RP{}; // μs
    double TAU_SRA{}; // μs
    double TAU_E{}; // μs
    double TAU_K{}; // μs
    double NU_K{}; // μs
    double MIN_NU_K{}; // μs
    double MIN_TAU_K{}; // μs

    double ETA_LTP{}; // mV
    double ETA_LTD{}; // mV
    double ETA_ILTP{}; // mV
    double ETA_ILTD{}; // mV
    double ETA_SR{}; // mV
    double DELTA_RP{}; // mv
    double DELTA_SRA{}; // mV
    double ETA_INH{}; // mV
    double ETA{}; // mV

    double VRESET{}; // mV
    double VTHRESH{}; // mV

    size_t SYNAPSE_DELAY{}; // μs

    double NORM_FACTOR{};
    double LATERAL_NORM_FACTOR{};
    double TOPDOWN_NORM_FACTOR{};
    double DECAY_RATE{};

    double TARGET_SPIKE_RATE{}; // spikes/s
    double MIN_THRESH{}; // mV

    std::string STDP_LEARNING{};
    std::string TRACKING{};
    std::vector<double> POTENTIAL_TRACK;

private:
    void loadSimpleNeuronsParameters(const std::string &fileName);

    void loadComplexNeuronsParameters(const std::string &fileName);

    void loadCriticNeuronsParameters(const std::string &fileName);

    void loadActorNeuronsParameters(const std::string &fileName);
};

class ReinforcementLearningConfig {
public:
    bool rlTraining{};
    double nu{};
    double V0{};
    double tauR{};
    double explorationFactor{};
    long actionRate{};
    long minActionRate{};
    double decayRate{};
    double scoreInterval{};
    bool intrinsicReward{};
    std::vector<std::pair<uint64_t, float>> actionMapping{};

    ReinforcementLearningConfig();

    explicit ReinforcementLearningConfig(const std::string& configFile);

    [[nodiscard]] bool getRLTraining() const { return rlTraining; }

    [[nodiscard]] std::vector<std::pair<uint64_t, float>> getActionMapping() const { return actionMapping; }

    [[nodiscard]] double getNu() const { return nu; }

    [[nodiscard]] double getV0() const { return V0; }

    [[nodiscard]] double getTauR() const { return tauR; }

    [[nodiscard]] double getExplorationFactor() const { return explorationFactor; }

    [[nodiscard]] long getActionRate() const { return actionRate; }

    [[nodiscard]] long getMinActionRate() const { return minActionRate; }

    [[nodiscard]] double getScoreInterval() const { return scoreInterval; }

    [[nodiscard]] double getDecayRate() const { return decayRate; }

    [[nodiscard]] bool getIntrinsicReward() const { return intrinsicReward; }

    void setExplorationFactor(double factor) { explorationFactor = factor; }

    void setActionRate(long rate) { actionRate = rate; }

private:
    void loadRLConfig(const std::string &fileName);
};

#endif //NEUVISYS_DV_CONFIG_HPP
