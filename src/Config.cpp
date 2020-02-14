#include "Config.hpp"

using json = nlohmann::json;

/***** Display parameters *****/
int X_NEURON = 0;
int Y_NEURON = 0;
int LAYER = 0;
int IND = X_NEURON*(NETWORK_HEIGHT+NETWORK_DEPTH) + Y_NEURON*LAYER + LAYER;

std::string FILE_NUMBER = "0";
std::string SAVE_DATA_LOCATION = "/home/thomas/Documents/Results/weights/";
std::string CONF_FILES_LOCATION = "/home/thomas/neuvisys-dv/configs/";

/***** Spiking Neural Network layout parameters *****/
int NEURON_WIDTH = 20;
int NEURON_HEIGHT = 20;

int X_ANCHOR_POINT = 0; // px
int Y_ANCHOR_POINT = 0; // px
int NETWORK_WIDTH = 10; // neurons
int NETWORK_HEIGHT = 10; // neurons
int NETWORK_DEPTH = 8; // neurons

/***** Neurons internal parameters *****/
double TAU_M = 10000; // μs
double TAU_LTP = 10000; // μs
double TAU_LTD = 20000; // μs
long INHIBITION = 20000; // μs
int SPEED = 500000; // μs

double VRESET = -20; // mV
double THRESHOLD = 15; // mV

double DELTA_VP = 0.05; // mV
double DELTA_VD = 0.03; // mV

double V_MIN = 4; // mV
double V_DEP = 3; // mV

double NORM_FACTOR = 4;
int NORM_THRESHOLD = 10; // number spikes

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

        TAU_M = conf["TAU_M"];
        TAU_LTP = conf["TAU_LTP"];
        TAU_LTD = conf["TAU_LTD"];
        SPEED = conf["SPEED"];
        VRESET = conf["VRESET"];
        THRESHOLD = conf["THRESHOLD"];
        DELTA_VP = conf["DELTA_VP"];
        DELTA_VD = conf["DELTA_VD"];
        NORM_FACTOR = conf["NORM_FACTOR"];
        NORM_THRESHOLD = conf["NORM_THRESHOLD"];
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
