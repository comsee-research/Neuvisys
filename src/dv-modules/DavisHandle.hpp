//
// Created by thomas on 24/11/2021.
//

#ifndef NEUVISYS_DAVISHANDLE_HPP
#define NEUVISYS_DAVISHANDLE_HPP

#include <libcaercpp/devices/davis.hpp>
#include <atomic>
#include <csignal>

int prepareCamera(libcaer::devices::davis &davis);
void changeBiases(libcaer::devices::davis &davis);
void eventLoop(libcaer::devices::davis &davis);

#endif //NEUVISYS_DAVISHANDLE_HPP
