//
// Created by thomas on 28/06/2021.
//

#include <motor-control/StepMotor.hpp>
#include "SimulationInterface.hpp"
#include "../dv-modules/DavisHandle.hpp"

static atomic_bool globalShutdown(false);

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

int launchLearningSimulation(std::string &networkPath) {
    NetworkHandle network(networkPath);

    SimulationInterface sim;
    sim.enableSyncMode(true);
    sim.startSimulation();

    std::vector<size_t> vecEvents;

    double actionTime = 0, updateTime = 0, consoleTime = 0;
    size_t iteration = 0;
    int action = 0;
    while (ros::ok() && sim.getSimulationTime() < 300) {
        sim.triggerNextTimeStep();
        while(!sim.simStepDone()) {
            ros::spinOnce();
        }

        sim.update();
        if (!sim.getLeftEvents().empty()) {
            network.transmitReward(sim.getReward());
            network.transmitEvents(sim.getLeftEvents());

            if (sim.getSimulationTime() - updateTime > static_cast<double>(UPDATE_INTERVAL) / E6) {
                updateTime = sim.getSimulationTime();
                network.updateNeuronStates(UPDATE_INTERVAL);
            }

            if (sim.getSimulationTime() - actionTime > static_cast<double>(network.getNetworkConfig().getActionRate()) / E6) {
                actionTime = sim.getSimulationTime();
                if (action != -1) {
                    network.updateActor(sim.getLeftEvents().back().timestamp(), action);
                }
                auto choice = network.actionSelection(network.resolveMotor(), network.getNetworkConfig().getExplorationFactor());
                action = choice.first;
                if (action != -1) {
                    sim.activateMotors(action);
                }
                network.saveActionMetrics(action, choice.second);
            }

            if (sim.getSimulationTime() - consoleTime > SCORE_INTERVAL) {
                consoleTime = sim.getSimulationTime();
                network.learningDecay(iteration);
                ++iteration;
                std::string msg = "Average reward: " + std::to_string(network.getScore(SCORE_INTERVAL * E3 / DT)) +
                                  "\nExploration factor: " + std::to_string(network.getNetworkConfig().getExplorationFactor()) +
                                  "\nAction rate: " + std::to_string(network.getNetworkConfig().getActionRate());
                std::cout << msg << std::endl;
            }
        }
    }

    sim.stopSimulation();
    network.save(1, "Simulation");
    return 0;
}

int launchLearningDavis(std::string &networkPath) {
    std::chrono::high_resolution_clock::time_point time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point updateTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point actionTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point consoleTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point motorTime = std::chrono::high_resolution_clock::now();

    StepMotor motor = StepMotor("leftmotor1", 0, "/dev/ttyUSB0");

    std::vector<double> motorMapping;
    motorMapping.emplace_back(350); // left horizontal -> left movement
    motorMapping.emplace_back(0); // no movement
    motorMapping.emplace_back(-350); // left horizontal  -> right movement

    NetworkHandle network(networkPath);

    // Install signal handler for global shutdown.
    struct sigaction shutdownAction;

    shutdownAction.sa_handler = &globalShutdownSignalHandler;
    shutdownAction.sa_flags = 0;
    sigemptyset(&shutdownAction.sa_mask);
    sigaddset(&shutdownAction.sa_mask, SIGTERM);
    sigaddset(&shutdownAction.sa_mask, SIGINT);

    if (sigaction(SIGTERM, &shutdownAction, NULL) == -1) {
        libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
                          "Failed to set signal handler for SIGTERM. Error: %d.", errno);
        return (EXIT_FAILURE);
    }

    if (sigaction(SIGINT, &shutdownAction, NULL) == -1) {
        libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
                          "Failed to set signal handler for SIGINT. Error: %d.", errno);
        return (EXIT_FAILURE);
    }

    // Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
    libcaer::devices::davis davis = libcaer::devices::davis(1);

    // Let's take a look at the information we have on the device.
    struct caer_davis_info davis_info = davis.infoGet();

    printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", davis_info.deviceString,
           davis_info.deviceID, davis_info.deviceIsMaster, davis_info.dvsSizeX, davis_info.dvsSizeY,
           davis_info.logicVersion);

    // Send the default configuration before using the device.
    // No configuration is sent automatically!
    davis.sendDefaultConfig();

    // Tweak some biases, to increase bandwidth in this case.
    struct caer_bias_coarsefine coarseFineBias;

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

    // Now let's get start getting some data from the device. We just loop in blocking mode,
    // no notification needed regarding new events. The shutdown notification, for example if
    // the device is disconnected, should be listened to.
    davis.dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);

    // Let's turn on blocking data-get mode to avoid wasting resources.
    davis.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
//    prepareCamera(davis);

    std::vector<size_t> vecEvents;

    size_t iteration = 0;
    double position = 0;
    int action = 0;
    while (!globalShutdown.load(memory_order_relaxed)) {
        std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = davis.dataGet();
        if (packetContainer == nullptr) {
            continue; // Skip if nothing there.
        }

        for (auto &packet : *packetContainer) {
            if (packet == nullptr) {
                continue; // Skip if nothing there.
            }

            if (packet->getEventType() == POLARITY_EVENT) {
                time = std::chrono::high_resolution_clock::now();
                std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
                        = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

                network.transmitReward(80 * (55000 - abs(position)) / 55000);
                network.saveValueMetrics(static_cast<double>(polarity->back().getTimestamp()), polarity->size());
                for (const auto &eve : *polarity) {
                    network.transmitEvent(Event(eve.getTimestamp(), eve.getX(), eve.getY(), eve.getPolarity(), 0));
                }

                if (std::chrono::duration<double>(time - updateTime).count() > static_cast<double>(UPDATE_INTERVAL) / E6) {
                    updateTime = std::chrono::high_resolution_clock::now();
                    network.updateNeuronStates(UPDATE_INTERVAL);
                }

                if (std::chrono::duration<double>(time - actionTime).count() > static_cast<double>(network.getNetworkConfig().getActionRate()) / E6) {
                    actionTime = std::chrono::high_resolution_clock::now();
                    if (action != -1) {
                        network.updateActor(polarity->back().getTimestamp(), action);
                    }
                    auto choice = network.actionSelection(network.resolveMotor(), network.getNetworkConfig().getExplorationFactor());
                    action = choice.first;
                    motor.setSpeed(motorMapping[action]);
                    network.saveActionMetrics(action, choice.second);
                }

                if (std::chrono::duration<double>(time - consoleTime).count() > SCORE_INTERVAL) {
                    consoleTime = std::chrono::high_resolution_clock::now();
                    network.learningDecay(iteration);
                    ++iteration;
                    std::string msg = "Average reward: " + std::to_string(network.getScore(SCORE_INTERVAL * E3 / DT)) +
                                      "\nExploration factor: " + std::to_string(network.getNetworkConfig().getExplorationFactor()) +
                                      "\nAction rate: " + std::to_string(network.getNetworkConfig().getActionRate());
                    std::cout << msg << std::endl;
                }

                if (std::chrono::duration<double>(time - motorTime).count() > 1) {
                    motorTime = std::chrono::high_resolution_clock::now();

                    position = motor.getPosition();
                    std::cout << position << std::endl;
                    if (position < -55000) {
                        std::cout << "overreach right" << std::endl;
                        motor.setSpeed(350);
                    } else if (position > 55000) {
                        std::cout << "overreach left" << std::endl;
                        motor.setSpeed(-350);
                    }
                }
            }
        }
    }
    davis.dataStop();

    // Close automatically done by destructor.
    printf("Shutdown successful.\n");
    network.save(1, "Simulation");
    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");

//    std::string type = argv[1];
    std::string type = "real";

    std::string networkPath;
    if (type == "multi") {
        for (const auto &entry : std::filesystem::directory_iterator(argv[1])) {
            networkPath = static_cast<std::string>(entry.path()) + "/configs/network_config.json";
            std::cout << networkPath << std::endl;
            launchLearningSimulation(networkPath);
        }
    } else if (type == "simulation") {
        networkPath = static_cast<std::string>(argv[2]) + "/configs/network_config.json";
        launchLearningSimulation(networkPath);
    }  else if (type == "real") {
//        networkPath = static_cast<std::string>(argv[2]) + "/configs/network_config.json";
        networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
        launchLearningDavis(networkPath);
    } else {
        networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
        launchLearningSimulation(networkPath);
    }
}