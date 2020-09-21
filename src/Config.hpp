#ifndef NEUVISYS_DV_CONFIG_HPP
#define NEUVISYS_DV_CONFIG_HPP

#include <opencv2/opencv.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"
#include "xtensor/xnpy.hpp"
#include "src/dependencies/json.hpp"

#include "Utils.hpp"

using json = nlohmann::json;

namespace Conf {
    /***** General parameters *****/
    inline constexpr int WIDTH = 346; // px
    inline constexpr int HEIGHT = 260; // px

    inline constexpr int EVENT_FREQUENCY = 1000; // μs
    inline constexpr int DISPLAY_FREQUENCY = 30000; // μs
    inline constexpr int UPDATE_PARAMETER_FREQUENCY = 1000000; // μs

    inline constexpr int TIME_WINDOW_SR = 20; // s

    inline const std::string CONF_FILE("/home/thomas/neuvisys-dv/configuration/conf.json");
    inline const std::string GUI_FILE("/home/thomas/neuvisys-dv/configuration/gui.json");
}

namespace Selection {
    inline int LAYER = 0;
    [[maybe_unused]] inline int LAYER2 = 0;
    inline int SYNAPSE = 0;
    inline unsigned int INDEX = 0;
    inline unsigned int INDEX2 = 0;
}

class NetworkConfig {
public:
    explicit NetworkConfig(std::string configFile);
    void loadConfiguration(std::string &fileName);
    void loadNetworkLayout(std::string &fileName);

    /***** Display parameters *****/
    std::string NETWORK_CONFIG;
    bool SaveData{};
    std::string SaveDataLocation;

    /***** Spiking Neural Network layout parameters *****/
    std::string Neuron1Config;
    std::string Neuron2Config;
    int L1Width{};
    int L1Height{};
    int L1Depth{};
    int L2Width{};
    int L2Height{};

    std::vector<int> L1XAnchor;
    std::vector<int> L1YAnchor;
    std::vector<int> L2XAnchor;
    std::vector<int> L2YAnchor;
    int Neuron1Width{};
    int Neuron1Height{};
    int Neuron1Synapses{};

    int Neuron2Width{};
    int Neuron2Height{};
    bool WeightSharing{};
};

class NeuronConfig {
public:
    NeuronConfig(std::string configFile, int type);
/***** Neurons internal parameters *****/
    double TAU_M{}; // μs
    double TAU_LTP{}; // μs
    double TAU_LTD{}; // μs
    double TAU_RP{}; // μs
    double TAU_SRA{}; // μs

    double DELTA_VP{}; // mV
    double DELTA_VD{}; // mV
    double DELTA_SR{}; // mV
    double DELTA_RP{}; // mv
    double DELTA_SRA{}; // mV
    double DELTA_INH{}; // mV

    double VRESET{}; // mV
    double VTHRESH{}; // mV

    long SYNAPSE_DELAY{}; // μs

    double NORM_FACTOR{};
    double DECAY_FACTOR{};

    double TARGET_SPIKE_RATE{}; // spikes/s
    double MIN_THRESH{}; // mV

    bool STDP_LEARNING{};
private:
    void loadNeuronsParameters(std::string &fileName);
    void loadPoolingNeuronsParameters(std::string &fileName);
};

#endif //NEUVISYS_DV_CONFIG_HPP
