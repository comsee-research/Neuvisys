//
// Created by thomas on 28/06/2021.
//

#include <motor-control/StepMotor.hpp>
#include "SimulationInterface.hpp"
#include "../dv-modules/DavisHandle.hpp"

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

    StepMotor m_motor = StepMotor("leftmotor1", 0, "/dev/ttyUSB0");
    NetworkHandle network(networkPath);

    libcaer::devices::davis davis(1);
    prepareCamera(davis);

    std::vector<size_t> vecEvents;

    size_t iteration = 0;
    int action = 0;
    while (iteration < E6) {
        std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = davis.dataGet();
        if (packetContainer == nullptr) {
            continue; // Skip if nothing there.
        }

        printf("\nGot event container with %d packets (allocated).\n", packetContainer->size());

        for (auto &packet : *packetContainer) {
            if (packet == nullptr) {
                continue; // Skip if nothing there.
            }

            if (packet->getEventType() == POLARITY_EVENT) {
                std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
                        = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

                network.transmitReward(0);
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

                    double position = m_motor.getPosition();
                    if (position < -100000) {
                        m_motor.setSpeed(250);
                    } else if (position > 100000) {
                        m_motor.setSpeed(-250);
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

    std::string networkPath;
    if (std::strcmp(argv[1], "multi") != 0) {
        for (const auto &entry : std::filesystem::directory_iterator(argv[1])) {
            networkPath = static_cast<std::string>(entry.path()) + "/configs/network_config.json";
            std::cout << networkPath << std::endl;
            launchLearningSimulation(networkPath);
        }
    } else if (std::strcmp(argv[1], "simulation") != 0) {
        networkPath = static_cast<std::string>(argv[2]) + "/configs/network_config.json";
        launchLearningSimulation(networkPath);
    }  else if (std::strcmp(argv[1], "real") != 0) {
        networkPath = static_cast<std::string>(argv[2]) + "/configs/network_config.json";
        launchLearningDavis(networkPath);
    } else {
        networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
        launchLearningSimulation(networkPath);
    }
}