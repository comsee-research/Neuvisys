#include "Config.hpp"

#include <utility>

using json = nlohmann::json;

NetworkConfig::NetworkConfig() = default;

NetworkConfig::NetworkConfig(const std::string &networkConfigPath) {
    m_networkConfigPath = networkConfigPath;
    std::string toRemove = "configs/network_config.json";
    m_networkPath = m_networkConfigPath.substr(0, m_networkConfigPath.size() - toRemove.size());
    loadNetworkLayout();
}

void NetworkConfig::loadNetworkLayout() {
    json conf;

    std::ifstream ifs(m_networkConfigPath);
    if (ifs.is_open()) {
        try {
            ifs >> conf;
            nbCameras = conf["nbCameras"];
            layerCellTypes = static_cast<std::vector<std::string>>(conf["layerCellTypes"]);
            layerInhibitions = static_cast<std::vector<std::vector<std::string>>>(conf["layerInhibitions"]);
            interLayerConnections = static_cast<std::vector<int>>(conf["interLayerConnections"]);
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
        } catch (const std::exception &e) {
            std::cerr << "In network config file:" << e.what() << std::endl;
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
            LATERAL_NORM_FACTOR = conf["LATERAL_NORM_FACTOR"];
            TOPDOWN_NORM_FACTOR = conf["TOPDOWN_NORM_FACTOR"];
            DECAY_RATE = conf["DECAY_RATE"];
            TARGET_SPIKE_RATE = conf["TARGET_SPIKE_RATE"];
            MIN_THRESH = conf["MIN_THRESH"];
            STDP_LEARNING = conf["STDP_LEARNING"];
            TRACKING = conf["TRACKING"];
            POTENTIAL_TRACK = static_cast<std::vector<double>>(conf["POTENTIAL_TRACK"]);
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
        //    TAU_SRA = E3 * static_cast<double>(conf["TAU_SRA"]);
            VTHRESH = conf["VTHRESH"];
            ETA_INH = conf["ETA_INH"];
            VRESET = conf["VRESET"];
            NORM_FACTOR = conf["NORM_FACTOR"];
            DECAY_RATE = conf["DECAY_RATE"];
            STDP_LEARNING = conf["STDP_LEARNING"];
            TRACKING = conf["TRACKING"];
            POTENTIAL_TRACK = static_cast<std::vector<double>>(conf["POTENTIAL_TRACK"]);
            DELTA_RP = conf["ETA_RP"];
        //    DELTA_SRA = conf["ETA_SRA"];
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
            POTENTIAL_TRACK = static_cast<std::vector<double>>(conf["POTENTIAL_TRACK"]);
            STDP_LEARNING = conf["STDP_LEARNING"];
            NORM_FACTOR = conf["NORM_FACTOR"];
            DECAY_RATE = conf["DECAY_RATE"];
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
            POTENTIAL_TRACK = static_cast<std::vector<double>>(conf["POTENTIAL_TRACK"]);
            STDP_LEARNING = conf["STDP_LEARNING"];
            NORM_FACTOR = conf["NORM_FACTOR"];
            DECAY_RATE = conf["DECAY_RATE"];
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
    std::filesystem::create_directory(directory + "/gabors");
    std::filesystem::create_directory(directory + "/gabors/0");
    std::filesystem::create_directory(directory + "/gabors/1");
    std::filesystem::create_directory(directory + "/gabors/2");
    std::filesystem::create_directory(directory + "/gabors/3");
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
                    {"layerCellTypes", {"SimpleCell", "ComplexCell"}},
                    {"layerInhibitions", {{"static"}, {"static"}}},
                    {"interLayerConnections", {-1, 0}},
                    {"layerPatches", {{{93}, {50}, {0}}, {{0}, {0}, {0}}}},
                    {"layerSizes",    {{16, 16, 64}, {4, 4, 16}}},
                    {"neuronSizes",       {{10, 10, 1}, {4, 4, 64}}},
                    {"neuronOverlap", {{0, 0, 0}, {0, 0, 0}}},
                    {"nu",            0.5},
                    {"V0",            0},
                    {"tauR",       1},
                    {"explorationFactor", 70},
                    {"actionRate", 500},
                    {"minActionRate", 100},
                    {"decayRate", 0.01},
            },
            {
                    {"VTHRESH",   30},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"POTENTIAL_TRACK",   {4, 10}},
                    {"TAU_SRA",        100},
                    {"TAU_RP",           30},
                    {"TAU_M",                 18},
                    {"TAU_LTP",      7},
                    {"TAU_LTD",       14},
                    {"TARGET_SPIKE_RATE", 0.75},
                    {"SYNAPSE_DELAY", 0},
                    {"STDP_LEARNING", "excitatory"},
                    {"NORM_FACTOR",   4},
                    {"LATERAL_NORM_FACTOR", 100},
                    {"TOPDOWN_NORM_FACTOR", 30},
                    {"DECAY_RATE", 0},
                    {"MIN_THRESH",        4},
                    {"ETA_LTP",    0.0077},
                    {"ETA_LTD",       -0.0021},
                    {"ETA_ILTP",  7.7},
                    {"ETA_ILTD", -2.1},
                    {"ETA_SRA", 0.6},
                    {"ETA_TA", 0},
                    {"ETA_RP", 1},
                    {"ETA_INH", 20},
            },
            {
                    {"VTHRESH",   3},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"POTENTIAL_TRACK",    {4,10}},
                    {"TAU_M",          20},
                    {"TAU_LTP",          20},
                    {"TAU_LTD",               20},
                    {"TAU_RP",       30},
                    {"STDP_LEARNING", "excitatory"},
                    {"NORM_FACTOR",       10},
                    {"DECAY_RATE",    0},
                    {"ETA_LTP",       0.2},
                    {"ETA_LTD",       0.2},
                    {"ETA_INH",    15},
                    {"ETA_RP",            1},
            },
            {
                    {"VTHRESH",   2},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"POTENTIAL_TRACK",    {4,10}},
                    {"TAU_M",          20},
                    {"ETA_INH",          0},
                    {"TAU_LTP",               7},
                    {"TAU_LTD",      14},
                    {"ETA_LTP",       0.077},
                    {"ETA_LTD",           -0.021},
                    {"NORM_FACTOR",   10},
                    {"DECAY_RATE",    0},
                    {"STDP_LEARNING", "all"},
                    {"NU_K",       200},
                    {"MIN_NU_K",          100},
                    {"TAU_K",      50},
                    {"MIN_TAU_K",     25},
                    {"TAU_E",     500},
                    {"ETA",      0.2}
            },
            {
                    {"VTHRESH",   2},
                    {"VRESET",          -20},
                    {"TRACKING",    "partial"},
                    {"POTENTIAL_TRACK",    {4, 10}},
                    {"TAU_M",          20},
                    {"ETA_INH",          0},
                    {"TAU_LTP",               7},
                    {"TAU_LTD",      14},
                    {"ETA_LTP",       0.077},
                    {"ETA_LTD",           -0.021},
                    {"NORM_FACTOR",   10},
                    {"DECAY_RATE",    0},
                    {"STDP_LEARNING", "all"},
                    {"TAU_E",      250},
                    {"ETA",               0.2}
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
