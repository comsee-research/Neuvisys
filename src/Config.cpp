#include "Config.hpp"

using json = nlohmann::json;

int NEURON_WIDTH = 8;
int NEURON_HEIGHT = 8;

double TAU_M = 10000; // μs
double TAU_LTP = 10000; // μs
double TAU_LTD = 20000; // μs

int SPEED = 500000; // μs

double VRESET = -20; // mV
double THRESHOLD = 15; // mV

double DELTA_VP = 0.05; // mV
double DELTA_VD = 0.03; // mV

double V_MIN = 4; // mV
double V_DEP = 3; // mV

double NORM_FACTOR = 4;
int NORM_THRESHOLD = 10; // number spikes

void Config::loadConfig(std::string &fileName) {
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        json conf;
        ifs >> conf;

        NEURON_WIDTH = conf["NEURON_WIDTH"];
        NEURON_HEIGHT = conf["NEURON_HEIGHT"];
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
        std::cout << "cannot open config file" << std::endl;
    }
    ifs.close();
}

void Config::saveConfig(std::string &fileName) {
    json conf;

    conf["NEURON_WIDTH"] = NEURON_WIDTH;
    conf["NEURON_HEIGHT"] = NEURON_HEIGHT;
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
        std::cout << "cannot open config file" << std::endl;
    }
    ofs.close();
}
