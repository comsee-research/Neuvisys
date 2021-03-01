#include "Config.hpp"

#include <utility>

using json = nlohmann::json;

NetworkConfig::NetworkConfig(std::string networkPath) {
    NETWORK_CONFIG = std::move(networkPath);
    loadNetworkLayout(NETWORK_CONFIG);
}

void NetworkConfig::loadNetworkLayout(const std::string& fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            NbCameras = conf["NbCameras"];
            L1Width = conf["L1Width"];
            L1Height = conf["L1Height"];
            L1Depth = conf["L1Depth"];
            L2Width = conf["L2Width"];
            L2Height = conf["L2Height"];
            L2Depth = conf["L2Depth"];

            for (const auto& x : conf["L1XAnchor"]) {
                L1XAnchor.push_back(x);
            }
            for (const auto& y : conf["L1YAnchor"]) {
                L1YAnchor.push_back(y);
            }
            for (const auto& x2 : conf["L2XAnchor"]) {
                L2XAnchor.push_back(x2);
            }
            for (const auto& y2 : conf["L2YAnchor"]) {
                L2YAnchor.push_back(y2);
            }
            Neuron1Width = conf["Neuron1Width"];
            Neuron1Height = conf["Neuron1Height"];
            Neuron1Synapses = conf["Neuron1Synapses"];

            Neuron2Width = conf["Neuron2Width"];
            Neuron2Height = conf["Neuron2Height"];
            Neuron2Depth = conf["Neuron2Depth"];

            SharingType = conf["SharingType"];
            SaveData = conf["SaveData"];

            std::string toErase = "configs/network_config.json";
            NetworkPath = fileName;
            NetworkPath.erase(fileName.find(toErase), toErase.length());
        } catch (const std::exception& e) {
            std::cerr << "In network config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open network configuration file" << std::endl;
    }
    ifs.close();
}

NeuronConfig::NeuronConfig(const std::string& configFile, size_t type) {
    if (type == 0) {
        loadNeuronsParameters(configFile);
    } else if (type == 1) {
        loadPoolingNeuronsParameters(configFile);
    }
}

void NeuronConfig::loadNeuronsParameters(const std::string& fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            ETA_LTP = conf["ETA_LTP"];
            ETA_LTD = conf["ETA_LTD"];
            ETA_SR = conf["ETA_TA"];
            DELTA_RP = conf["ETA_RP"];
            DELTA_SRA = conf["ETA_SRA"];
            DELTA_INH = conf["ETA_INH"];
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
            STDP_LEARNING = conf["STDP_LEARNING"];
            TRACKING = conf["TRACKING"];
        } catch (const std::exception& e) {
            std::cerr << "In simple cell config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}

void NeuronConfig::loadPoolingNeuronsParameters(const std::string& fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            ETA_LTP = conf["ETA_LTP"];
            ETA_LTD = conf["ETA_LTD"];
            TAU_LTP = conf["TAU_LTP"];
            TAU_LTD = conf["TAU_LTD"];
            TAU_M = conf["TAU_M"];
            VTHRESH = conf["VTHRESH"];
            DELTA_INH = conf["ETA_INH"];
            VRESET = conf["VRESET"];
            NORM_FACTOR = conf["NORM_FACTOR"];
            DECAY_FACTOR = conf["DECAY_FACTOR"];
            STDP_LEARNING = conf["STDP_LEARNING"];
            TRACKING = conf["TRACKING"];
            DELTA_RP = conf["ETA_RP"];
            TAU_RP = conf["TAU_RP"];
        } catch (const std::exception& e) {
            std::cerr << "In complex cell config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}
