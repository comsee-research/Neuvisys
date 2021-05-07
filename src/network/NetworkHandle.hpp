//
// Created by alphat on 06/05/2021.
//

#ifndef NEUVISYS_DV_NETWORK_HANDLE_HPP
#define NEUVISYS_DV_NETWORK_HANDLE_HPP

#include "SpikingNetwork.hpp"

std::vector<Event> mono(const std::string& events, size_t nbPass);
std::vector<Event> stereo(const std::string& events, size_t nbPass);
void multiplePass(const std::string& networkPath, const std::string& events, size_t nbPass);

#endif //NEUVISYS_DV_NETWORK_HANDLE_HPP
