#ifndef NEUVISYS_DV_CONFIG_H
#define NEUVISYS_DV_CONFIG_H

#include <opencv2/opencv.hpp>
#include "xtensor/xtensor.hpp"
#include "xtensor-blas/xlinalg.hpp"

#include "Utils.hpp"

const int WIDTH = 346;
const int HEIGHT = 260;
const int NEURON_WIDTH = 260;
const int NEURON_HEIGHT = 260;

const int ADJACENT_NEURONS = 4;
const unsigned int NUMBER_THREADS = 4;

const double DECAY = 0.1f;
const double GAIN = 1e4 * DECAY / (0.2 * NEURON_WIDTH * NEURON_HEIGHT);
const int SPEED = 500000;

const xt::xarray<double> NO_GABOR = GAIN * xt::ones<double>({NEURON_HEIGHT, NEURON_WIDTH});
const xt::xarray<double> GABOR_H = opencvMatToXarray(GAIN * cv::getGaborKernel(cv::Size(NEURON_WIDTH, NEURON_HEIGHT), 60, 0, 1, 8, 0), NEURON_HEIGHT, NEURON_WIDTH); // horizontal gabor
const xt::xarray<double> GABOR_V = opencvMatToXarray(GAIN * cv::getGaborKernel(cv::Size(NEURON_WIDTH, NEURON_HEIGHT), 60, M_PI/2, 1, 8, 0), NEURON_HEIGHT, NEURON_WIDTH); // vertical gabor

const xt::xarray<long> NO_DELAYS = xt::zeros<long>({NEURON_HEIGHT, NEURON_WIDTH});
const xt::xarray<long> DELAYS_LR = xt::linalg::outer(xt::ones<long>({NEURON_HEIGHT}), xt::linspace<long>(SPEED, 0, NEURON_WIDTH)); // left to right
const xt::xarray<long> DELAYS_RL = xt::linalg::outer(xt::ones<long>({NEURON_HEIGHT}), xt::linspace<long>(0, SPEED, NEURON_WIDTH)); // right to left
const xt::xarray<long> DELAYS_TB = xt::linalg::outer(xt::linspace<long>(SPEED, 0, NEURON_HEIGHT), xt::ones<long>({NEURON_WIDTH})); // top to bottom
const xt::xarray<long> DELAYS_BT = xt::linalg::outer(xt::linspace<long>(0, SPEED, NEURON_HEIGHT), xt::ones<long>({NEURON_WIDTH})); // bottom to top

#endif //NEUVISYS_DV_CONFIG_H
