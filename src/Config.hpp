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

    inline const std::string CONF_FILE = "/home/thomas/neuvisys-dv/configuration/conf.json";
}

namespace Selection {
    inline int X_NEURON;
    inline int Y_NEURON;
    inline int LAYER;
    inline int SYNAPSE;
    inline unsigned int IND;

    inline int NET_WIDTH = 36;
    inline int NET_HEIGHT = 24;
    inline int NET_DEPTH = 30;
    inline int NET_SYNAPSES = 1;
};

class NetworkConfig {
public:
    explicit NetworkConfig(std::string configFile);
    void loadConfiguration(std::string &fileName);
    void loadNetworkLayout(std::string &fileName);

    /***** Display parameters *****/
    bool SAVE_DATA;
    bool WEIGHT_SHARING;
    std::string SAVE_DATA_LOCATION;
    std::string CONF_FILES_LOCATION;

/***** Spiking Neural Network layout parameters *****/
    int NEURON_WIDTH;
    int NEURON_HEIGHT;
    int NEURON_SYNAPSES;

    int X_ANCHOR_POINT;
    int Y_ANCHOR_POINT;
    int NETWORK_WIDTH;
    int NETWORK_HEIGHT;
    int NETWORK_DEPTH;
};

class NeuronConfig {
public:
    explicit NeuronConfig(std::string configFile);
/***** Neurons internal parameters *****/
    double TAU_M; // μs
    double TAU_LTP; // μs
    double TAU_LTD; // μs
    double TAU_RP; // μs
    double TAU_SRA; // μs

    double DELTA_VP; // mV
    double DELTA_VD; // mV
    double DELTA_SR; // mV
    double DELTA_RP; // mv
    double DELTA_SRA; // mV
    double DELTA_INH; // mV

    double VRESET; // mV
    double VTHRESH; // mV

    long SYNAPSE_DELAY; // μs

    double NORM_FACTOR;
    double DECAY_FACTOR;

    double TARGET_SPIKE_RATE; // spikes/s
private:
    void loadNeuronsParameters(std::string &fileName);
};

#endif //NEUVISYS_DV_CONFIG_HPP
