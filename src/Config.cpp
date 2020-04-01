#include "Config.hpp"

using json = nlohmann::json;

/***** Display parameters *****/
int X_NEURON;
int Y_NEURON;
int LAYER;
int SYNAPSE;
int IND;

bool SAVE_DATA;
std::string SAVE_DATA_LOCATION;
std::string CONF_FILES_LOCATION;

/***** Spiking Neural Network layout parameters *****/
int NEURON_WIDTH;
int NEURON_HEIGHT;
int NEURON_SYNAPSES;

int X_ANCHOR_POINT; // px
int Y_ANCHOR_POINT; // px
int NETWORK_WIDTH; // neurons
int NETWORK_HEIGHT; // neurons
int NETWORK_DEPTH; // neurons

/***** Neurons internal parameters *****/
double TAU_M; // μs
double TAU_LTP; // μs
double TAU_LTD; // μs
double TAU_INHIB; // μs

double VRESET; // mV
double VTHRESH; // mV

double DELTA_VP; // mV
double DELTA_VD; // mV
double DELTA_SR; // mV

long SYNAPSE_DELAY; // μs

double NORM_FACTOR;
int NORM_THRESHOLD; // number spikes

double TARGET_SPIKE_RATE; // spikes/s

void Config::loadConfiguration(std::string &fileName) {
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        json conf;
        ifs >> conf;

        SAVE_DATA = conf["SAVE_DATA"];
        SAVE_DATA_LOCATION = conf["SAVE_DATA_LOCATION"];
        CONF_FILES_LOCATION = conf["CONF_FILES_LOCATION"];
    } else {
        std::cout << "cannot open main configuration file" << std::endl;
    }
    ifs.close();
}

void Config::loadNeuronsParameters(std::string &fileName) {
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        json conf;
        ifs >> conf;

        DELTA_VP = conf["DELTA_VP"];
        DELTA_VD = conf["DELTA_VD"];
        DELTA_SR = conf["DELTA_SR"];
        TAU_LTP = conf["TAU_LTP"];
        TAU_LTD = conf["TAU_LTD"];
        VTHRESH = conf["VTHRESH"];
        VRESET = conf["VRESET"];
        TAU_M = conf["TAU_M"];
        TAU_INHIB = conf["TAU_INHIB"];
        SYNAPSE_DELAY = conf["SYNAPSE_DELAY"];

        NORM_FACTOR = conf["NORM_FACTOR"];
        NORM_THRESHOLD = conf["NORM_THRESHOLD"];
        TARGET_SPIKE_RATE = conf["TARGET_SPIKE_RATE"];
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}

void Config::loadNetworkLayout(std::string &fileName) {
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        json conf;
        ifs >> conf;

        NEURON_WIDTH = conf["NEURON_WIDTH"];
        NEURON_HEIGHT = conf["NEURON_HEIGHT"];
        NEURON_SYNAPSES = conf["NEURON_SYNAPSES"];

        X_ANCHOR_POINT = conf["X_ANCHOR_POINT"];
        Y_ANCHOR_POINT = conf["Y_ANCHOR_POINT"];
        NETWORK_WIDTH = conf["NETWORK_WIDTH"];
        NETWORK_HEIGHT = conf["NETWORK_HEIGHT"];
        NETWORK_DEPTH = conf["NETWORK_DEPTH"];
    } else {
        std::cout << "cannot open network configuration file" << std::endl;
    }
    ifs.close();
}
