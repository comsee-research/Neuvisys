#include "Config.hpp"

using json = nlohmann::json;

/***** Display parameters *****/
int X_NEURON;
int Y_NEURON;
int LAYER;
int SYNAPSE;
int IND;

std::string FILE_NUMBER;
std::string SAVE_DATA_LOCATION;
std::string CONF_FILES_LOCATION;

/***** Spiking Neural Network layout parameters *****/
int NEURON_WIDTH;
int NEURON_HEIGHT;

int X_ANCHOR_POINT; // px
int Y_ANCHOR_POINT; // px
int NETWORK_WIDTH; // neurons
int NETWORK_HEIGHT; // neurons
int NETWORK_DEPTH; // neurons

/***** Neurons internal parameters *****/
double TAU_M; // μs
double TAU_LTP; // μs
double TAU_LTD; // μs
long INHIBITION; // μs
int SPEED; // μs

double VRESET; // mV
double THRESHOLD; // mV

double DELTA_VP; // mV
double DELTA_VD; // mV

double NORM_FACTOR;
int NORM_THRESHOLD; // number spikes

void Config::loadConfiguration(std::string &fileName) {
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        json conf;
        ifs >> conf;

        FILE_NUMBER = conf["FILE_NUMBER"];
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

        DELTA_VD = conf["DELTA_VD"];
        DELTA_VP = conf["DELTA_VP"];
        INHIBITION = conf["INHIBITION"];
        NORM_FACTOR = conf["NORM_FACTOR"];
        NORM_THRESHOLD = conf["NORM_THRESHOLD"];
        SPEED = conf["SPEED"];
        TAU_LTD = conf["TAU_LTD"];
        TAU_LTP = conf["TAU_LTP"];
        TAU_M = conf["TAU_M"];
        THRESHOLD = conf["THRESHOLD"];
        VRESET = conf["VRESET"];
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

void Config::saveNeuronsParameters(std::string &fileName) {
    json conf;

    conf["TAU_M"] = TAU_M;
    conf["TAU_LTP"] = TAU_LTP;
    conf["TAU_LTD"] = TAU_LTD;
    conf["SPEED"] = SPEED;
    conf["VRESET"] = VRESET;
    conf["THRESHOLD"] = THRESHOLD;
    conf["DELTA_VP"] = DELTA_VP;
    conf["DELTA_VD"] = DELTA_VD;
    conf["NORM_FACTOR"] = NORM_FACTOR;
    conf["NORM_THRESHOLD"] = NORM_THRESHOLD;

    std::ofstream ofs(fileName);
    if (ofs.is_open()) {
        ofs << std::setw(4) << conf << std::endl;
    } else {
        std::cout << "cannot open and save neuron configuration file" << std::endl;
    }
    ofs.close();
}
