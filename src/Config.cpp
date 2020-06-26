#include "Config.hpp"

using json = nlohmann::json;

NetworkConfig::NetworkConfig(std::string configFile) {
    loadConfiguration(configFile);
    loadNetworkLayout(NETWORK_CONFIG);
}

void NetworkConfig::loadConfiguration(std::string &fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            SAVE_DATA = conf["SAVE_DATA"];
            SAVE_DATA_LOCATION = conf["SAVE_DATA_LOCATION"];
            NETWORK_CONFIG = conf["NETWORK_CONFIG"];
        } catch (const std::exception& e) {
            std::cerr << "In Main config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open main configuration file" << std::endl;
    }
    ifs.close();
}

void NetworkConfig::loadNetworkLayout(std::string &fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
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

            WeightSharing = conf["WeightSharing"];
        } catch (const std::exception& e) {
            std::cerr << "In Network config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open network configuration file" << std::endl;
    }
    ifs.close();
}

NeuronConfig::NeuronConfig(std::string configFile, int type) {
    if (type == 0) {
        loadNeuronsParameters(configFile);
    } else if (type == 1) {
        loadPoolingNeuronsParameters(configFile);
    }
}

void NeuronConfig::loadNeuronsParameters(std::string &fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
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
            MIN_THRESH = conf["MIN_THRESH"];
        } catch (const std::exception& e) {
            std::cerr << "In Neuron config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}

void NeuronConfig::loadPoolingNeuronsParameters(std::string &fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            DELTA_VP = conf["DELTA_VP"];
            TAU_LTP = conf["TAU_LTP"];
            TAU_M = conf["TAU_M"];
            VTHRESH = conf["VTHRESH"];
            VRESET = conf["VRESET"];
            NORM_FACTOR = conf["NORM_FACTOR"];
        } catch (const std::exception& e) {
            std::cerr << "In Pooling Neuron config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}