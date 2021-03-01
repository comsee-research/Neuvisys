#ifndef NEUVISYS_DV_CONFIG_HPP
#define NEUVISYS_DV_CONFIG_HPP

#include <iostream>
#include <fstream>
#include "src/dependencies/json.hpp"

using json = nlohmann::json;

namespace Conf {
    /***** General parameters *****/
    inline constexpr size_t WIDTH = 346; // px
    inline constexpr size_t HEIGHT = 260; // px

    inline constexpr size_t EVENT_FREQUENCY = 1000; // μs
    inline constexpr size_t UPDATE_PARAMETER_FREQUENCY = 1000000; // μs

    inline constexpr size_t TIME_WINDOW_SR = 20; // s
}

class NetworkConfig {
public:
    explicit NetworkConfig(std::string networkPath);
    void loadNetworkLayout(const std::string& fileName);

    /***** Display parameters *****/
    std::string NETWORK_CONFIG;
    bool SaveData{};
    std::string NetworkPath;

    /***** Spiking Neural Network layout parameters *****/
    size_t NbCameras{};
    size_t L1Width{};
    size_t L1Height{};
    size_t L1Depth{};
    size_t L2Width{};
    size_t L2Height{};
    size_t L2Depth{};

    std::vector<size_t> L1XAnchor;
    std::vector<size_t> L1YAnchor;
    std::vector<size_t> L2XAnchor;
    std::vector<size_t> L2YAnchor;
    size_t Neuron1Width{};
    size_t Neuron1Height{};
    size_t Neuron1Synapses{};

    size_t Neuron2Width{};
    size_t Neuron2Height{};
    size_t Neuron2Depth{};
    std::string SharingType{};
};

class NeuronConfig {
public:
    NeuronConfig(const std::string& configFile, size_t type);
/***** Neurons internal parameters *****/
    double TAU_M{}; // μs
    double TAU_LTP{}; // μs
    double TAU_LTD{}; // μs
    double TAU_RP{}; // μs
    double TAU_SRA{}; // μs

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
};

#endif //NEUVISYS_DV_CONFIG_HPP
