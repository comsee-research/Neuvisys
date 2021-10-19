#ifndef NEUVISYS_DV_CONFIG_HPP
#define NEUVISYS_DV_CONFIG_HPP

#include <iostream>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include "../dependencies/json.hpp"

using json = nlohmann::json;

namespace Conf {
    /***** General parameters *****/
    inline constexpr size_t WIDTH = 1024; // px
    inline constexpr size_t HEIGHT = 1024; // px

    inline constexpr size_t EVENT_FREQUENCY = 1000; // μs
    inline constexpr size_t UPDATE_PARAMETER_FREQUENCY = 1000000; // μs

    inline constexpr size_t TIME_WINDOW_SR = 20; // s
    inline constexpr size_t E6 = 1000000;
    inline constexpr size_t E3 = 1000;
}

class NetworkConfig {
    /***** Display parameters *****/
    std::string NETWORK_CONFIG;
    bool saveData{};
    std::string NetworkPath;

    /***** Spiking Neural Network layout parameters *****/
    size_t nbCameras{};
    size_t neuron1Synapses{};
    std::string sharingType{};

    std::vector<std::string> layerCellTypes;
    std::vector<bool> layerInhibitions;
    std::vector<size_t> interLayerConnections;
    std::vector<std::vector<std::vector<size_t>>> layerPatches;
    std::vector<std::vector<size_t>> layerSizes;
    std::vector<std::vector<size_t>> neuronSizes;
    std::vector<std::vector<size_t>> neuronOverlap;

    double nu{};
    double V0{};
    double tauR{};
    double explorationFactor{};
    long actionRate{};
    double decayRate{};

public:
    NetworkConfig();
    explicit NetworkConfig(std::string networkPath);
    void loadNetworkLayout(const std::string& fileName);

    [[nodiscard]] std::string getNetworkPath() const { return NetworkPath; }
    [[nodiscard]] size_t getNbCameras() const { return nbCameras; }
    [[nodiscard]] size_t getNeuron1Synapses() const { return neuron1Synapses; }
    [[nodiscard]] bool getSaveData() const { return saveData; }
    std::string &getNetworkPath() { return NetworkPath; }
    std::string &getSharingType() { return sharingType; }
    [[nodiscard]] double getNu() const { return nu; }
    [[nodiscard]] double getV0() const { return V0; }
    [[nodiscard]] double getTauR() const { return tauR; }
    [[nodiscard]] double getExplorationFactor() const { return explorationFactor; }
    [[nodiscard]] long getActionRate() const { return actionRate; }
    [[nodiscard]] double getDecayRate() const { return decayRate; }
    std::vector<std::string> &getLayerCellTypes() { return layerCellTypes; }
    std::vector<bool> &getLayerInhibitions() { return layerInhibitions; }
    std::vector<size_t> &getInterLayerConnections() { return interLayerConnections; }
    std::vector<std::vector<std::vector<size_t>>> getLayerPatches() { return layerPatches; }
    std::vector<std::vector<size_t>> getLayerSizes() { return layerSizes; }
    std::vector<std::vector<size_t>> getNeuronSizes() { return neuronSizes; }
    std::vector<std::vector<size_t>> getNeuronOverlap() { return neuronOverlap; }
    void setExplorationFactor(double factor) { explorationFactor = factor; }
    void setActionRate(long rate) { actionRate = rate; }
    static void createNetwork(const std::string& directory);
};

class NeuronConfig {
public:
    NeuronConfig();
    NeuronConfig(const std::string& configFile, size_t type);
/***** Neurons internal parameters *****/
    double TAU_M{}; // μs
    double TAU_LTP{}; // μs
    double TAU_LTD{}; // μs
    double TAU_RP{}; // μs
    double TAU_SRA{}; // μs
    double TAU_E{}; // μs
    double TAU_K{}; // μs
    double NU_K{}; // μs

    double ETA_LTP{}; // mV
    double ETA_LTD{}; // mV
    double ETA_SR{}; // mV
    double DELTA_RP{}; // mv
    double DELTA_SRA{}; // mV
    double ETA_INH{}; // mV
    double ETA{}; // mV

    double VRESET{}; // mV
    double VTHRESH{}; // mV

    size_t SYNAPSE_DELAY{}; // μs

    double NORM_FACTOR{};

    double TARGET_SPIKE_RATE{}; // spikes/s
    double MIN_THRESH{}; // mV

    bool STDP_LEARNING{};
    std::string TRACKING{};
private:
    void loadSimpleNeuronsParameters(const std::string& fileName);
    void loadComplexNeuronsParameters(const std::string& fileName);
    void loadCriticNeuronsParameters(const std::string& fileName);
    void loadActorNeuronsParameters(const std::string& fileName);
};

#endif //NEUVISYS_DV_CONFIG_HPP
