#ifndef NEUVISYS_DV_CONFIG_HPP
#define NEUVISYS_DV_CONFIG_HPP

#include <opencv2/opencv.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"
#include "xtensor/xnpy.hpp"
#include "src/dependencies/json.hpp"

#include "Utils.hpp"

class Config {
public:
    static void loadConfiguration(std::string &fileName);
    static void loadNeuronsParameters(std::string &fileName);
    static void loadNetworkLayout(std::string &fileName);
};

/***** General parameters *****/
const int WIDTH = 346; // px
const int HEIGHT = 260; // px

const int EVENT_FREQUENCY = 1000; // μs
const int DISPLAY_FREQUENCY = 30000; // μs
const int UPDATE_PARAMETER_FREQUENCY = 1000000; // μs

const int TIME_WINDOW_SR = 20; // s

const std::string CONF_FILE = "/home/thomas/neuvisys-dv/configuration/conf.json";

/***** Display parameters *****/
extern bool SAVE_DATA;
extern bool WEIGHT_SHARING;
extern std::string SAVE_DATA_LOCATION;
extern std::string CONF_FILES_LOCATION;

extern int X_NEURON;
extern int Y_NEURON;
extern int LAYER;
extern int SYNAPSE;
extern int IND;

/***** Spiking Neural Network layout parameters *****/
extern int NEURON_WIDTH;
extern int NEURON_HEIGHT;
extern int NEURON_SYNAPSES;

extern int X_ANCHOR_POINT;
extern int Y_ANCHOR_POINT;
extern int NETWORK_WIDTH;
extern int NETWORK_HEIGHT;
extern int NETWORK_DEPTH;

/***** Neurons internal parameters *****/
extern double TAU_M; // μs
extern double TAU_LTP; // μs
extern double TAU_LTD; // μs
extern double TAU_RP; // μs
extern double TAU_SRA; // μs

extern double DELTA_VP; // mV
extern double DELTA_VD; // mV
extern double DELTA_SR; // mV
extern double DELTA_RP; // mv
extern double DELTA_SRA; // mV
extern double DELTA_INH; // mV

extern double VRESET; // mV
extern double VTHRESH; // mV

extern long SYNAPSE_DELAY; // μs

extern double NORM_FACTOR;
extern double DECAY_FACTOR;

extern double TARGET_SPIKE_RATE; // spikes/s

#endif //NEUVISYS_DV_CONFIG_HPP
