//
// Created by thomas on 23/03/2022.
//

#ifndef NEUVISYS_EVENTCAMERA_HPP
#define NEUVISYS_EVENTCAMERA_HPP

#include <atomic>
#include <libcaercpp/devices/davis.hpp>
#include <csignal>

class EventCamera {
    libcaer::devices::davis davis = libcaer::devices::davis(1);

public:
    EventCamera();

    ~EventCamera();

    void changeBiases();

    std::shared_ptr<const libcaer::events::PolarityEventPacket> receiveEvents(bool &received, bool &stop);

private:
    static int prepareContext();
};

#endif //NEUVISYS_EVENTCAMERA_HPP
