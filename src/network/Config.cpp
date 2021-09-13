#include "Config.hpp"

#include <utility>

using json = nlohmann::json;

NetworkConfig::NetworkConfig() = default;

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

            for (const auto &size : conf["layerPatches"]) {
                layerPatches.push_back(size);
            }
            for (const auto &size : conf["layerSizes"]) {
                layerSizes.push_back(size);
            }
            for (const auto &size : conf["neuronSizes"]) {
                neuronSizes.push_back(size);
            }

            Neuron1Synapses = conf["Neuron1Synapses"];
            SharingType = conf["SharingType"];
            SaveData = conf["SaveData"];
            NU = conf["NU"];
            V0 = conf["V0"];
            TAU_R = conf["TAU_R"];

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

NeuronConfig::NeuronConfig() = default;


NeuronConfig::NeuronConfig(const std::string& configFile, size_t type) {
    if (type == 0) {
        loadNeuronsParameters(configFile);
    } else if (type == 1) {
        loadPoolingNeuronsParameters(configFile);
    } else if (type == 2) {
        loadMotorNeuronsParameters(configFile);
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
            ETA_INH = conf["ETA_INH"];
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
            ETA_INH = conf["ETA_INH"];
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

void NeuronConfig::loadMotorNeuronsParameters(const std::string& fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            TAU_M = conf["TAU_M"];
            TAU_E = conf["TAU_E"];
            TAU_K = conf["TAU_K"];
            NU_K = conf["NU_K"];
            ETA = conf["ETA"];
            VTHRESH = conf["VTHRESH"];
            ETA_INH = conf["ETA_INH"];
            VRESET = conf["VRESET"];
            TRACKING = conf["TRACKING"];
            TAU_LTP = conf["TAU_LTP"];
            TAU_LTD = conf["TAU_LTD"];
            ETA_LTP = conf["ETA_LTP"];
            ETA_LTD = conf["ETA_LTD"];
            STDP_LEARNING = conf["STDP_LEARNING"];
            NORM_FACTOR = conf["NORM_FACTOR"];
        } catch (const std::exception& e) {
            std::cerr << "In motor cell config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}
