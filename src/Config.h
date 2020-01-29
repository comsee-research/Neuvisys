#ifndef NEUVISYS_DV_CONFIG_H
#define NEUVISYS_DV_CONFIG_H

#include <opencv2/opencv.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"

#include "Utils.hpp"

const int WIDTH = 346;
const int HEIGHT = 260;
const int NEURON_WIDTH = 16;
const int NEURON_HEIGHT = 16;

const int NUMBER_DISPLAY = 2;
const int NUMBER_THREADS = 4;

const double TAU_M = 10000; // μs
const double TAU_LTP = 10000; // μs
const double TAU_LTD = 20000; // μs

const int SPEED = 500000; // μs

const double VRESET = -20; // mV
const double THRESHOLD = 20; // mV
const double DELTA_VP = 0.05; // mV
const double DELTA_VD = 0.03; // mV

const double GAIN = 1e-3 * TAU_M / (NEURON_WIDTH * NEURON_HEIGHT);

const int NORMALIZATION_THRESHOLD = 10;

const xt::xarray<double> NO_GABOR = xt::ones<double>({NEURON_HEIGHT, NEURON_WIDTH, 2});
const xt::xarray<double> GABOR_H = opencvMatToXarray(cv::getGaborKernel(cv::Size(NEURON_WIDTH, NEURON_HEIGHT), 50, M_PI/2, 1.06, 8, M_PI/2), NEURON_HEIGHT, NEURON_WIDTH); // horizontal gabor
const xt::xarray<double> GABOR_V = opencvMatToXarray(cv::getGaborKernel(cv::Size(NEURON_WIDTH, NEURON_HEIGHT), 50, 0, 1.06, 8, M_PI/2), NEURON_HEIGHT, NEURON_WIDTH); // vertical gabor
const xt::xarray<double> UNIFORM_WEIGHTS = uniformMatrix(NEURON_HEIGHT, NEURON_WIDTH);

const xt::xarray<long> NO_DELAYS = xt::zeros<long>({NEURON_HEIGHT, NEURON_WIDTH});
const xt::xarray<long> DELAYS_LR = xt::linalg::outer(xt::ones<long>({NEURON_HEIGHT}), xt::linspace<long>(SPEED, 0, NEURON_WIDTH)); // left to right
const xt::xarray<long> DELAYS_RL = xt::linalg::outer(xt::ones<long>({NEURON_HEIGHT}), xt::linspace<long>(0, SPEED, NEURON_WIDTH)); // right to left
const xt::xarray<long> DELAYS_TB = xt::linalg::outer(xt::linspace<long>(SPEED, 0, NEURON_HEIGHT), xt::ones<long>({NEURON_WIDTH})); // top to bottom
const xt::xarray<long> DELAYS_BT = xt::linalg::outer(xt::linspace<long>(0, SPEED, NEURON_HEIGHT), xt::ones<long>({NEURON_WIDTH})); // bottom to top

#endif //NEUVISYS_DV_CONFIG_H
