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
            saveData = conf["saveData"];
            NU = conf["NU"];
            V0 = conf["V0"];
            TAU_R = conf["TAU_R"];
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
        loadNeuronsParameters(configFile);
    } else if (type == 1) {
        loadPoolingNeuronsParameters(configFile);
    } else if (type == 2) {
        loadMotorNeuronsParameters(configFile);
    }
}

void NeuronConfig::loadNeuronsParameters(const std::string &fileName) {
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
        } catch (const std::exception &e) {
            std::cerr << "In simple cell config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}

void NeuronConfig::loadPoolingNeuronsParameters(const std::string &fileName) {
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
        } catch (const std::exception &e) {
            std::cerr << "In complex cell config file" << std::endl;
            throw;
        }
    } else {
        std::cout << "cannot open neuron configuration file" << std::endl;
    }
    ifs.close();
}

void NeuronConfig::loadMotorNeuronsParameters(const std::string &fileName) {
    json conf;

    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            TAU_M = conf["TAU_M"];
            TAU_E = conf["TAU_E"];
            TAU_K = static_cast<double>(conf["TAU_K"]) / Conf::E6;
            NU_K = static_cast<double>(conf["NU_K"]) / Conf::E6;
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
    std::filesystem::create_directory(directory + "/figures/complex_directions");
    std::filesystem::create_directory(directory + "/figures/complex_figures");
    std::filesystem::create_directory(directory + "/figures/complex_orientations");
    std::filesystem::create_directory(directory + "/figures/complex_weights_orientations");
    std::filesystem::create_directory(directory + "/figures/motor_figures");
    std::filesystem::create_directory(directory + "/figures/simple_figures");
    std::filesystem::create_directory(directory + "/gabors");
    std::filesystem::create_directory(directory + "/gabors/data");
    std::filesystem::create_directory(directory + "/gabors/figures");
    std::filesystem::create_directory(directory + "/gabors/hists");
    std::filesystem::create_directory(directory + "/images");
    std::filesystem::create_directory(directory + "/images/simple_cells");
    std::filesystem::create_directory(directory + "/images/complex_cells");
    std::filesystem::create_directory(directory + "/weights");
    std::filesystem::create_directory(directory + "/weights/simple_cells");
    std::filesystem::create_directory(directory + "/weights/complex_cells");
    std::filesystem::create_directory(directory + "/weights/critic_cells");
    std::filesystem::create_directory(directory + "/weights/actor_cells");

    std::vector<json> conf = {
            {
                    {"nbCameras", 1},
                    {"neuron1Synapses", 1},
                    {"sharingType", "patch"},
                    {"saveData", true},
                    {"layerCellTypes", {"SimpleCell", "ComplexCell", "CriticCell", "ActorCell"}},
                    {"layerInhibitions", {true, true, false, false}},
                    {"interLayerConnections", {0, 0, 1, 1}},
                    {"layerPatches", {{{33}, {110}, {0}}, {{0}, {0}, {0}}, {{0}, {0}, {0}}, {{0}, {0}, {0}}}},
                    {"layerSizes", {{28, 4, 64}, {27, 1, 16}, {100, 1, 1}, {2, 1, 1}}},
                    {"neuronSizes",   {{10, 10, 1}, {4, 4, 64}, {27, 1, 16}, {27, 1, 16}}},
                    {"neuronOverlap", {{0, 0, 0}, {3, 3, 0}, {0, 0, 0}, {0, 0, 0}}},
                    {"NU",                2},
                    {"V0",            0},
                    {"TAU_R",         1}},
            {
                    {"VTHRESH",   30},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"TAU_SRA",  100000},
                    {"TAU_RP",       20000},
                    {"TAU_M",      18000},
                    {"TAU_LTP",       7000},
                    {"TAU_LTD",      14000},
                    {"TARGET_SPIKE_RATE", 0.75},
                    {"SYNAPSE_DELAY", 0},
                    {"STDP_LEARNING", true},
                    {"NORM_FACTOR", 4},
                    {"MIN_THRESH", 4},
                    {"ETA_LTP",      0.0077},
                    {"ETA_LTD", -0.0021},
                    {"ETA_SRA", 0.6},
                    {"ETA_TA", 1},
                    {"ETA_RP", 1},
                    {"ETA_INH", 20},
                    {"DECAY_FACTOR", 0}},
            {
                    {"VTHRESH",   3},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"TAU_M",    20000},
                    {"TAU_LTP",      20000},
                    {"TAU_LTD",    20000},
                    {"STDP_LEARNING", true},
                    {"NORM_FACTOR",   10},
                    {"ETA_LTP",           0.2},
                    {"ETA_LTD",       0.2},
                    {"ETA_INH",       25},
                    {"ETA_RP",      1},
                    {"TAU_RP",     20000},
                    {"DECAY_FACTOR", 0}},
            {
                    {"VTHRESH",   2},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"TAU_M",    20000},
                    {"ETA_INH",      25},
                    {"TAU_LTP",    7000},
                    {"TAU_LTD",       14000},
                    {"ETA_LTP",       0.077},
                    {"ETA_LTD",           -0.021},
                    {"NORM_FACTOR",   10},
                    {"STDP_LEARNING", true},
                    {"DELTA_INH",   10},
                    {"NU_K",       400000},
                    {"TAU_K",        100000},
                    {"TAU_E",   500000},
                    {"ETA",     0.025}},
            {
                    {"VTHRESH",   2},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"TAU_M",    20000},
                    {"ETA_INH",      25},
                    {"TAU_LTP",    7000},
                    {"TAU_LTD",       14000},
                    {"ETA_LTP",       0.077},
                    {"ETA_LTD",           -0.021},
                    {"NORM_FACTOR",   10},
                    {"STDP_LEARNING", true},
                    {"DELTA_INH",   10},
                    {"NU_K",       400000},
                    {"TAU_K",        100000},
                    {"TAU_E",   500000},
                    {"ETA",     0.05}}
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
