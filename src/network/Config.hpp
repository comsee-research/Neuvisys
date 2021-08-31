#ifndef NEUVISYS_DV_CONFIG_HPP
#define NEUVISYS_DV_CONFIG_HPP

#include <iostream>
#include <fstream>
#include "../dependencies/json.hpp"

using json = nlohmann::json;

namespace Conf {
    /***** General parameters *****/
    inline constexpr size_t WIDTH = 346; // px
    inline constexpr size_t HEIGHT = 260; // px

    inline constexpr size_t EVENT_FREQUENCY = 1000; // μs
    inline constexpr size_t UPDATE_PARAMETER_FREQUENCY = 1000000; // μs

    inline constexpr size_t TIME_WINDOW_SR = 20; // s

    /***** TD-LTP *****/
    inline constexpr double nu = 2; //
    inline constexpr double V0 = 0; // mV

    inline constexpr double tau_k = 0.2; // s
    inline constexpr double nu_k = 0.05; // s
    inline constexpr double tau_r = 1; // s
    inline constexpr double tau_e = 0.5; // s

    inline constexpr double eta = 0.025; //
}

class NetworkConfig {
    /***** Display parameters *****/
    std::string NETWORK_CONFIG;
    bool SaveData{};
    std::string NetworkPath;

    /***** Spiking Neural Network layout parameters *****/
    size_t NbCameras{};
    size_t Neuron1Synapses{};
    std::string SharingType{};

    std::vector<std::vector<std::vector<size_t>>> layerPatches;
    std::vector<std::vector<size_t>> layerSizes;
    std::vector<std::vector<size_t>> neuronSizes;

public:
    NetworkConfig();
    explicit NetworkConfig(std::string networkPath);
    void loadNetworkLayout(const std::string& fileName);

    [[nodiscard]] std::string getNetworkPath() const { return NetworkPath; }
    [[nodiscard]] size_t getNbCameras() const { return NbCameras; }
    [[nodiscard]] size_t getNeuron1Synapses() const { return Neuron1Synapses; }
    [[nodiscard]] bool getSaveData() const { return SaveData; }
    std::string &getNetworkPath() { return NetworkPath; }
    std::string &getSharingType() { return SharingType; }
    std::vector<std::vector<std::vector<size_t>>> &getLayerPatches() { return layerPatches; }
    std::vector<std::vector<size_t>> &getLayerSizes() { return layerSizes; }
    std::vector<std::vector<size_t>> &getNeuronSizes() { return neuronSizes; }
};

class NeuronConfig {
public:
    NeuronConfig();
    NeuronConfig(const std::string& configFile, size_t type);
/***** Neurons internal parameters *****/
    double TAU_M = 1; // μs
    double TAU_LTP = 1; // μs
    double TAU_LTD = 1; // μs
    double TAU_RP = 1; // μs
    double TAU_SRA = 1; // μs
    double TAU_E = 1; // μs

    double ETA_LTP{}; // mV
    double ETA_LTD{}; // mV
    double ETA_SR{}; // mV
    double DELTA_RP{}; // mv
    double DELTA_SRA{}; // mV
    double DELTA_INH{}; // mV

    double VRESET{}; // mV
    double VTHRESH{}; // mV

    size_t SYNAPSE_DELAY{}; // μs

    double NORM_FACTOR{};
    double DECAY_FACTOR{};

    double TARGET_SPIKE_RATE{}; // spikes/s
    double MIN_THRESH{}; // mV

    bool STDP_LEARNING{};
    std::string TRACKING{};
private:
    void loadNeuronsParameters(const std::string& fileName);
    void loadPoolingNeuronsParameters(const std::string& fileName);
    void loadMotorNeuronsParameters(const std::string& fileName);
};

#endif //NEUVISYS_DV_CONFIG_HPP
