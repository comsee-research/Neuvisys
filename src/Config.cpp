#include "Config.hpp"

using json = nlohmann::json;

NetworkConfig::NetworkConfig(std::string configFile) {
    loadConfiguration(configFile);
    loadNetworkLayout(NETWORK_CONFIG);
}

void NetworkConfig::loadConfiguration(std::string &fileName) {
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        json conf;
        ifs >> conf;

        SAVE_DATA = conf["SAVE_DATA"];
        WEIGHT_SHARING = conf["WEIGHT_SHARING"];
        SAVE_DATA_LOCATION = conf["SAVE_DATA_LOCATION"];
        NETWORK_CONFIG = conf["NETWORK_CONFIG"];
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

        Neuron1Config = conf["Neuron1Config"];
        Neuron2Config = conf["Neuron2Config"];
        L1Width = conf["L1Width"];
        L1Height = conf["L1Height"];
        L1Depth = conf["L1Depth"];
        L2Width = conf["L2Width"];
        L2Height = conf["L2Height"];

        L1XAnchor = conf["L1XAnchor"];
        L1YAnchor = conf["L1YAnchor"];
        Neuron1Width = conf["Neuron1Width"];
        Neuron1Height = conf["Neuron1Height"];
        Neuron1Synapses = conf["Neuron1Synapses"];

        Neuron2Width = conf["Neuron2Width"];
        Neuron2Height = conf["Neuron2Height"];

    } else {
        std::cout << "cannot open network configuration file" << std::endl;
    }
    ifs.close();

    Selection::NET_WIDTH = L1Width;
    Selection::NET_HEIGHT = L1Height;
    Selection::NET_DEPTH = L1Depth;
    Selection::NET_SYNAPSES = Neuron1Synapses;
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
