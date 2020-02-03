#ifndef NEUVISYS_DV_CONFIG_HPP
#define NEUVISYS_DV_CONFIG_HPP

#include <opencv2/opencv.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"
#include "src/Dependencies/json.hpp"

#include "Utils.hpp"

class Config {
public:
    static void loadConfig(std::string &fileName);
    static void saveConfig(std::string &fileName);
};

const int WIDTH = 346;
const int HEIGHT = 260;
extern int NEURON_WIDTH;
extern int NEURON_HEIGHT;

const int EVENT_FREQUENCY = 1000; // μs
const int DISPLAY_FREQUENCY = 30000; // μs

const int NUMBER_DISPLAY = 2;
const int NUMBER_THREADS = 4;

extern double TAU_M; // μs
extern double TAU_LTP; // μs
extern double TAU_LTD; // μs

extern int SPEED; // μs

extern double VRESET; // mV
extern double THRESHOLD; // mV

extern double DELTA_VP; // mV
extern double DELTA_VD; // mV

extern double V_MIN; // mV
extern double V_DEP; // mV

extern double NORM_FACTOR;
extern int NORM_THRESHOLD;

const xt::xarray<double> NO_GABOR = xt::ones<double>({NEURON_HEIGHT, NEURON_WIDTH, 2});
const xt::xarray<double> GABOR_H = opencvMatToXarray(cv::getGaborKernel(cv::Size(NEURON_WIDTH, NEURON_HEIGHT), 50, M_PI/2, 1.06, 8, M_PI/2), NEURON_HEIGHT, NEURON_WIDTH); // horizontal gabor
const xt::xarray<double> GABOR_V = opencvMatToXarray(cv::getGaborKernel(cv::Size(NEURON_WIDTH, NEURON_HEIGHT), 50, 0, 1.06, 8, M_PI/2), NEURON_HEIGHT, NEURON_WIDTH); // vertical gabor
const xt::xarray<double> UNIFORM_WEIGHTS = uniformMatrix(NEURON_HEIGHT, NEURON_WIDTH);

const xt::xarray<long> NO_DELAYS = xt::zeros<long>({NEURON_HEIGHT, NEURON_WIDTH});
const xt::xarray<long> DELAYS_LR = xt::linalg::outer(xt::ones<long>({NEURON_HEIGHT}), xt::linspace<long>(SPEED, 0, NEURON_WIDTH)); // left to right
const xt::xarray<long> DELAYS_RL = xt::linalg::outer(xt::ones<long>({NEURON_HEIGHT}), xt::linspace<long>(0, SPEED, NEURON_WIDTH)); // right to left
const xt::xarray<long> DELAYS_TB = xt::linalg::outer(xt::linspace<long>(SPEED, 0, NEURON_HEIGHT), xt::ones<long>({NEURON_WIDTH})); // top to bottom
const xt::xarray<long> DELAYS_BT = xt::linalg::outer(xt::linspace<long>(0, SPEED, NEURON_HEIGHT), xt::ones<long>({NEURON_WIDTH})); // bottom to top

#endif //NEUVISYS_DV_CONFIG_HPP
