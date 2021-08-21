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
    inline constexpr double nu = 2;
    inline constexpr double V0 = -40;

    inline constexpr long tau_k = 200000;
    inline constexpr long nu_k = 50000;

    inline constexpr double tau_r = 4000000;
}

class NetworkConfig {
public:
    NetworkConfig();
    explicit NetworkConfig(std::string networkPath);
    void loadNetworkLayout(const std::string& fileName);

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
