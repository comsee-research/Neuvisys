#include "Config.hpp"

#include <utility>

using json = nlohmann::json;

NetworkConfig::NetworkConfig() = default;

NetworkConfig::NetworkConfig(std::string networkPath) {
    NETWORK_CONFIG = std::move(networkPath);
    loadNetworkLayout(NETWORK_CONFIG);
}

void NetworkConfig::loadNetworkLayout(const std::string &fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            nbCameras = conf["nbCameras"];
            layerCellTypes = static_cast<std::vector<std::string>>(conf["layerCellTypes"]);
            layerInhibitions = static_cast<std::vector<bool>>(conf["layerInhibitions"]);
            interLayerConnections = static_cast<std::vector<size_t>>(conf["interLayerConnections"]);
            layerPatches = static_cast<std::vector<std::vector<std::vector<size_t>>>>(conf["layerPatches"]);
            layerSizes = static_cast<std::vector<std::vector<size_t>>>(conf["layerSizes"]);
            neuronSizes = static_cast<std::vector<std::vector<size_t>>>(conf["neuronSizes"]);
            neuronOverlap = static_cast<std::vector<std::vector<size_t>>>(conf["neuronOverlap"]);
            neuron1Synapses = conf["neuron1Synapses"];
            sharingType = conf["sharingType"];
            nu = conf["nu"];
            V0 = conf["V0"];
            tauR = conf["tauR"];
            explorationFactor = conf["explorationFactor"];
            decayRate = conf["decayRate"];
            actionRate = static_cast<long>(E3) * static_cast<long>(conf["actionRate"]);
            minActionRate = static_cast<long>(E3) * static_cast<long>(conf["minActionRate"]);
            std::string toErase = "configs/network_config.json";
            NetworkPath = fileName;
            NetworkPath.erase(fileName.find(toErase), toErase.length());
        } catch (const std::exception &e) {
            std::cerr << "In network config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open network configuration file" << std::endl;
    }
    ifs.close();
}

NeuronConfig::NeuronConfig() = default;

NeuronConfig::NeuronConfig(const std::string &configFile, size_t type) {
    if (type == 0) {
        loadSimpleNeuronsParameters(configFile);
    } else if (type == 1) {
        loadComplexNeuronsParameters(configFile);
    } else if (type == 2) {
        loadCriticNeuronsParameters(configFile);
    } else if (type == 3) {
        loadActorNeuronsParameters(configFile);
    }
}

void NeuronConfig::loadSimpleNeuronsParameters(const std::string &fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            ETA_LTP = conf["ETA_LTP"];
            ETA_LTD = conf["ETA_LTD"];
            ETA_ILTP = conf["ETA_ILTP"];
            ETA_ILTD = conf["ETA_ILTD"];
            ETA_SR = conf["ETA_TA"];
            DELTA_RP = conf["ETA_RP"];
            DELTA_SRA = conf["ETA_SRA"];
            ETA_INH = conf["ETA_INH"];
            TAU_LTP = E3 * static_cast<double>(conf["TAU_LTP"]);
            TAU_LTD = E3 * static_cast<double>(conf["TAU_LTD"]);
            TAU_RP = E3 * static_cast<double>(conf["TAU_RP"]);
            TAU_M = E3 * static_cast<double>(conf["TAU_M"]);
            TAU_SRA = E3 * static_cast<double>(conf["TAU_SRA"]);
            VTHRESH = conf["VTHRESH"];
            VRESET = conf["VRESET"];
            SYNAPSE_DELAY = conf["SYNAPSE_DELAY"];
            NORM_FACTOR = conf["NORM_FACTOR"];
            TARGET_SPIKE_RATE = conf["TARGET_SPIKE_RATE"];
            MIN_THRESH = conf["MIN_THRESH"];
            STDP_LEARNING = conf["STDP_LEARNING"];
            TRACKING = conf["TRACKING"];
        } catch (const std::exception &e) {
            std::cerr << "In simple cell config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}

void NeuronConfig::loadComplexNeuronsParameters(const std::string &fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            ETA_LTP = conf["ETA_LTP"];
            ETA_LTD = conf["ETA_LTD"];
            TAU_LTP = E3 * static_cast<double>(conf["TAU_LTP"]);
            TAU_LTD = E3 * static_cast<double>(conf["TAU_LTD"]);
            TAU_M = E3 * static_cast<double>(conf["TAU_M"]);
            TAU_RP = E3 * static_cast<double>(conf["TAU_RP"]);
            VTHRESH = conf["VTHRESH"];
            ETA_INH = conf["ETA_INH"];
            VRESET = conf["VRESET"];
            NORM_FACTOR = conf["NORM_FACTOR"];
            STDP_LEARNING = conf["STDP_LEARNING"];
            TRACKING = conf["TRACKING"];
            DELTA_RP = conf["ETA_RP"];
        } catch (const std::exception &e) {
            std::cerr << "In complex cell config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}

void NeuronConfig::loadCriticNeuronsParameters(const std::string &fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            ETA_LTP = conf["ETA_LTP"];
            ETA_LTD = conf["ETA_LTD"];
            TAU_M = E3 * static_cast<double>(conf["TAU_M"]);
            TAU_E = E3 * static_cast<double>(conf["TAU_E"]);
            TAU_LTP = E3 * static_cast<double>(conf["TAU_LTP"]);
            TAU_LTD = E3 * static_cast<double>(conf["TAU_LTD"]);
            TAU_K = static_cast<double>(conf["TAU_K"]) / E3;
            NU_K = static_cast<double>(conf["NU_K"]) / E3;
            MIN_TAU_K = static_cast<double>(conf["MIN_TAU_K"]) / E3;
            MIN_NU_K = static_cast<double>(conf["MIN_NU_K"]) / E3;
            ETA = conf["ETA"];
            VTHRESH = conf["VTHRESH"];
            ETA_INH = conf["ETA_INH"];
            VRESET = conf["VRESET"];
            TRACKING = conf["TRACKING"];
            STDP_LEARNING = conf["STDP_LEARNING"];
            NORM_FACTOR = conf["NORM_FACTOR"];
        } catch (const std::exception &e) {
            std::cerr << "In motor cell config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}

void NeuronConfig::loadActorNeuronsParameters(const std::string &fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            ETA_LTP = conf["ETA_LTP"];
            ETA_LTD = conf["ETA_LTD"];
            TAU_M = E3 * static_cast<double>(conf["TAU_M"]);
            TAU_E = E3 * static_cast<double>(conf["TAU_E"]);
            TAU_LTP = E3 * static_cast<double>(conf["TAU_LTP"]);
            TAU_LTD = E3 * static_cast<double>(conf["TAU_LTD"]);
            ETA = conf["ETA"];
            VTHRESH = conf["VTHRESH"];
            ETA_INH = conf["ETA_INH"];
            VRESET = conf["VRESET"];
            TRACKING = conf["TRACKING"];
            STDP_LEARNING = conf["STDP_LEARNING"];
            NORM_FACTOR = conf["NORM_FACTOR"];
        } catch (const std::exception &e) {
            std::cerr << "In motor cell config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}

void NetworkConfig::createNetwork(const std::string &directory) {
    std::filesystem::create_directory(directory);
    std::filesystem::create_directory(directory + "/configs");
    std::filesystem::create_directory(directory + "/figures");
    std::filesystem::create_directory(directory + "/figures/0");
    std::filesystem::create_directory(directory + "/figures/1");
    std::filesystem::create_directory(directory + "/figures/2");
    std::filesystem::create_directory(directory + "/figures/3");
    std::filesystem::create_directory(directory + "/figures/complex_directions");
    std::filesystem::create_directory(directory + "/figures/complex_orientations");
    std::filesystem::create_directory(directory + "/figures/complex_weights_orientations");
    std::filesystem::create_directory(directory + "/gabors");
    std::filesystem::create_directory(directory + "/gabors/data");
    std::filesystem::create_directory(directory + "/gabors/figures");
    std::filesystem::create_directory(directory + "/gabors/hists");
    std::filesystem::create_directory(directory + "/images");
    std::filesystem::create_directory(directory + "/images/0");
    std::filesystem::create_directory(directory + "/images/1");
    std::filesystem::create_directory(directory + "/images/2");
    std::filesystem::create_directory(directory + "/images/3");
    std::filesystem::create_directory(directory + "/weights");
    std::filesystem::create_directory(directory + "/weights/0");
    std::filesystem::create_directory(directory + "/weights/1");
    std::filesystem::create_directory(directory + "/weights/2");
    std::filesystem::create_directory(directory + "/weights/3");

    std::vector<json> conf = {
            {
                    {"nbCameras", 1},
                    {"neuron1Synapses", 1},
                    {"sharingType", "patch"},
                    {"saveData", true},
                    {"layerCellTypes", {"SimpleCell", "ComplexCell", "CriticCell", "ActorCell"}},
                    {"layerInhibitions", {true, true, false, false}},
                    {"interLayerConnections", {0, 0, 1, 1}},
                    {"layerPatches",  {{{33}, {110}, {0}}, {{0}, {0}, {0}}, {{0}, {0}, {0}}, {{0}, {0}, {0}}}},
                    {"layerSizes",        {{28, 4, 64}, {13, 1, 16}, {100, 1, 1}, {2, 1, 1}}},
                    {"neuronSizes",   {{10, 10, 1}, {4, 4, 64}, {13, 1, 16}, {13, 1, 16}}},
                    {"neuronOverlap", {{0, 0, 0}, {2, 2, 0}, {0, 0, 0}, {0, 0, 0}}},
                    {"nu",          0.5},
                    {"V0",         0},
                    {"tauR",    1},
                    {"explorationFactor", 70},
                    {"actionRate", 500},
                    {"minActionRate", 100},
                    {"decayRate", 0.01},
            },
            {
                    {"VTHRESH",   30},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"TAU_SRA",  100},
                    {"TAU_RP",         30},
                    {"TAU_M",            18},
                    {"TAU_LTP",               7},
                    {"TAU_LTD",       14},
                    {"TARGET_SPIKE_RATE", 0.75},
                    {"SYNAPSE_DELAY", 0},
                    {"STDP_LEARNING", "all"},
                    {"NORM_FACTOR", 4},
                    {"MIN_THRESH", 4},
                    {"ETA_LTP", 0.0077},
                    {"ETA_LTD",           -0.0021},
                    {"ETA_ILTP",   7.7},
                    {"ETA_ILTD",      -2.1},
                    {"ETA_SRA",   0.6},
                    {"ETA_TA", 0},
                    {"ETA_RP", 1},
                    {"ETA_INH", 20},
            },
            {
                    {"VTHRESH",   3},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"TAU_M",    20},
                    {"TAU_LTP",        20},
                    {"TAU_LTD",          20},
                    {"TAU_RP",                30},
                    {"STDP_LEARNING", "all"},
                    {"NORM_FACTOR",       10},
                    {"ETA_LTP",       0.2},
                    {"ETA_LTD",       0.2},
                    {"ETA_INH",     15},
                    {"ETA_RP",     1},
            },
            {
                    {"VTHRESH",   2},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"TAU_M",    20},
                    {"ETA_INH",        0},
                    {"TAU_LTP",          7},
                    {"TAU_LTD",               14},
                    {"ETA_LTP",       0.077},
                    {"ETA_LTD",           -0.021},
                    {"NORM_FACTOR",   10},
                    {"STDP_LEARNING", "all"},
                    {"NU_K",        200},
                    {"MIN_NU_K",   100},
                    {"TAU_K",   50},
                    {"MIN_TAU_K",         25},
                    {"TAU_E",      500},
                    {"ETA",           0.2}
            },
            {
                    {"VTHRESH",   2},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"TAU_M",    20},
                    {"ETA_INH",        0},
                    {"TAU_LTP",          7},
                    {"TAU_LTD",               14},
                    {"ETA_LTP",       0.077},
                    {"ETA_LTD",           -0.021},
                    {"NORM_FACTOR",   10},
                    {"STDP_LEARNING", "all"},
                    {"TAU_E",       250},
                    {"ETA",        0.2}
            }
    };
    size_t count = 0;
    for (auto file: {"configs/network_config.json", "configs/simple_cell_config.json",
                     "configs/complex_cell_config.json",
                     "configs/critic_cell_config.json", "configs/actor_cell_config.json"}) {
        std::ofstream ofs(directory + "/" + file);
        if (ofs.is_open()) {
            ofs << std::setw(4) << conf[count] << std::endl;
        } else {
            std::cout << "cannot create network files" << std::endl;
        }
        ofs.close();
        ++count;
    }
}
