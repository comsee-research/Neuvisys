//
// Created by thomas on 23/03/2022.
//

#include <iostream>
#include "EventCamera.hpp"

static std::atomic_bool globalShutdown(false);

static void globalShutdownSignalHandler(int signal) {
    // Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
    if (signal == SIGTERM || signal == SIGINT) {
        globalShutdown.store(true);
    }
}

static void usbShutdownHandler(void *ptr) {
    (void) (ptr); // UNUSED.

    globalShutdown.store(true);
}

int EventCamera::prepareContext() {
    // Install signal handler for global shutdown.
    struct sigaction shutdownAction{};

    shutdownAction.sa_handler = &globalShutdownSignalHandler;
    shutdownAction.sa_flags = 0;
    sigemptyset(&shutdownAction.sa_mask);
    sigaddset(&shutdownAction.sa_mask, SIGTERM);
    sigaddset(&shutdownAction.sa_mask, SIGINT);

    if (sigaction(SIGTERM, &shutdownAction, nullptr) == -1) {
        libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
                          "Failed to set signal handler for SIGTERM. Error: %d.", errno);
        return (EXIT_FAILURE);
    }

    if (sigaction(SIGINT, &shutdownAction, nullptr) == -1) {
        libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
                          "Failed to set signal handler for SIGINT. Error: %d.", errno);
        return (EXIT_FAILURE);
    }
    return 0;
}

void EventCamera::changeBiases() {
    // Tweak some biases, to increase bandwidth in this case.
    struct caer_bias_coarsefine coarseFineBias{};

    coarseFineBias.coarseValue = 2;
    coarseFineBias.fineValue = 116;
    coarseFineBias.enabled = true;
    coarseFineBias.sexN = false;
    coarseFineBias.typeNormal = true;
    coarseFineBias.currentLevelNormal = true;

    davis.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias));

    coarseFineBias.coarseValue = 1;
    coarseFineBias.fineValue = 33;
    coarseFineBias.enabled = true;
    coarseFineBias.sexN = false;
    coarseFineBias.typeNormal = true;
    coarseFineBias.currentLevelNormal = true;

    davis.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias));

    // Let's verify they really changed!
    uint32_t prBias = davis.configGet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP);
    uint32_t prsfBias = davis.configGet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP);

    printf("New bias values --- PR-coarse: %d, PR-fine: %d, PRSF-coarse: %d, PRSF-fine: %d.\n",
           caerBiasCoarseFineParse(prBias).coarseValue, caerBiasCoarseFineParse(prBias).fineValue,
           caerBiasCoarseFineParse(prsfBias).coarseValue, caerBiasCoarseFineParse(prsfBias).fineValue);
}

std::shared_ptr<const libcaer::events::PolarityEventPacket> EventCamera::receiveEvents(bool &received, bool &stop) {
    if (!globalShutdown.load(std::memory_order_relaxed)) {
        std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = davis.dataGet();
        if (packetContainer != nullptr) {
            for (auto &packet: *packetContainer) {
                if (packet != nullptr) {
                    if (packet->getEventType() == POLARITY_EVENT) {
                        std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(
                                packet);
                        received = true;
                        return polarity;
                    }
                }
            }
        }
    } else {
        stop = true;
    }
}

EventCamera::EventCamera() {
    prepareContext();

    struct caer_davis_info davis_info = davis.infoGet();
    printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", davis_info.deviceString,
           davis_info.deviceID, davis_info.deviceIsMaster, davis_info.dvsSizeX, davis_info.dvsSizeY,
           davis_info.logicVersion);
    davis.sendDefaultConfig();
    davis.dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);
    davis.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
}

EventCamera::~EventCamera() {
    davis.dataStop();
}
