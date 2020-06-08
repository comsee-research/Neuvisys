#include "Config.hpp"

using json = nlohmann::json;

NetworkConfig::NetworkConfig(std::string configFile) {
    loadConfiguration(configFile);
    loadNetworkLayout(CONF_FILES_LOCATION);
}

void NetworkConfig::loadConfiguration(std::string &fileName) {
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        json conf;
        ifs >> conf;

        SAVE_DATA = conf["SAVE_DATA"];
        WEIGHT_SHARING = conf["WEIGHT_SHARING"];
        SAVE_DATA_LOCATION = conf["SAVE_DATA_LOCATION"];
        CONF_FILES_LOCATION = conf["CONF_FILES_LOCATION"];
    } else {
        std::cout << "cannot open main configuration file" << std::endl;
    }
    ifs.close();
}

void NetworkConfig::loadNetworkLayout(std::string &fileName) {
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

NeuronConfig::NeuronConfig(std::string configFile) {
    loadNeuronsParameters(configFile);
}

void NeuronConfig::loadNeuronsParameters(std::string &fileName) {
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        json conf;
        ifs >> conf;

        DELTA_VP = conf["DELTA_VP"];
        DELTA_VD = conf["DELTA_VD"];
        DELTA_SR = conf["DELTA_SR"];
        DELTA_RP = conf["DELTA_RP"];
        DELTA_SRA = conf["DELTA_SRA"];
        DELTA_INH = conf["DELTA_INH"];
        TAU_LTP = conf["TAU_LTP"];
        TAU_LTD = conf["TAU_LTD"];
        TAU_RP = conf["TAU_RP"];
        TAU_M = conf["TAU_M"];
        TAU_SRA = conf["TAU_SRA"];
        VTHRESH = conf["VTHRESH"];
        VRESET = conf["VRESET"];
        SYNAPSE_DELAY = conf["SYNAPSE_DELAY"];
        NORM_FACTOR = conf["NORM_FACTOR"];
        DECAY_FACTOR = conf["DECAY_FACTOR"];
        TARGET_SPIKE_RATE = conf["TARGET_SPIKE_RATE"];
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}
