#include "Config.hpp"
#include "src/Dependencies/json.hpp"

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
    std::ifstream i(fileName);
    json j;
    i >> j;
}

void Config::saveConfig(std::string &fileName) {
    json j;

    j["NEURON_WIDTH"] = NEURON_WIDTH;
    j["NEURON_HEIGHT"] = NEURON_HEIGHT;
    j["TAU_M"] = TAU_M;
    j["TAU_LTP"] = TAU_LTP;
    j["TAU_LTD"] = TAU_LTD;
    j["SPEED"] = SPEED;
    j["VRESET"] = VRESET;
    j["THRESHOLD"] = THRESHOLD;
    j["DELTA_VP"] = DELTA_VP;
    j["DELTA_VD"] = DELTA_VD;
    j["NORM_FACTOR"] = NORM_FACTOR;
    j["NORM_THRESHOLD"] = NORM_THRESHOLD;

    std::ofstream o(fileName);
    o << std::setw(4) << j << std::endl;
}
