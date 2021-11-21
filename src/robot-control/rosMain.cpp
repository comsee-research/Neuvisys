//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

int launchLearning(std::string &networkPath) {
    NetworkHandle network(networkPath);

    SimulationInterface sim;
    sim.enableSyncMode(true);
    sim.startSimulation();

    std::vector<size_t> vecEvents;

    double actionTime = 0, updateTime = 0, consoleTime = 0;
    size_t iteration = 0;
    int actor = 0;
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
                if (actor != -1) {
                    network.updateActor(sim.getLeftEvents().back().timestamp(), actor);
                }
                sim.actionSelection(network.resolveMotor(), network.getNetworkConfig().getExplorationFactor(), actor);
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");

    std::string networkPath;
    if (argc > 2) {
        for (const auto &entry : std::filesystem::directory_iterator(argv[1])) {
            networkPath = static_cast<std::string>(entry.path()) + "/configs/network_config.json";
            std::cout << networkPath << std::endl;
            launchLearning(networkPath);
        }
    } else if (argc > 1) {
        networkPath = static_cast<std::string>(argv[1]) + "/configs/network_config.json";
    } else {
        networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    }
    launchLearning(networkPath);
}