//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

int launchLearning(std::string &networkPath) {
    NetworkHandle network(networkPath);


    SimulationInterface sim(1. / 150);
    sim.enableSyncMode(true);
    sim.startSimulation();

    std::vector<size_t> vecEvents;

    double actionTime = 0, consoleTime = 0;
    size_t iteration = 0;
    int actor = 0;
    while (ros::ok() && sim.getSimulationTime() < 300) {
        sim.triggerNextTimeStep();
        while(!sim.simStepDone()) {
            ros::spinOnce();
        }

        sim.update();
        network.transmitReward(sim.getReward());
        network.transmitEvents(sim.getLeftEvents());

        if (sim.getSimulationTime() - actionTime > static_cast<double>(network.getNetworkConfig().getActionRate()) / Conf::E6) {
            actionTime = sim.getSimulationTime();
            if (actor != -1) {
                network.updateActor(sim.getLeftEvents().back().timestamp(), actor);
            }
            sim.motorAction(network.resolveMotor(), network.getNetworkConfig().getExplorationFactor(), actor);
        }

        if (sim.getSimulationTime() - consoleTime > 10) {
            consoleTime = sim.getSimulationTime();
            network.learningDecay(iteration);
            ++iteration;
            std::string msg = "Average reward: " + std::to_string(network.getScore(1000)) +
                    "\nExploration factor: " + std::to_string(network.getNetworkConfig().getExplorationFactor()) +
                    "\nAction rate: " + std::to_string(network.getNetworkConfig().getActionRate()) +
                    "\nETA: " + std::to_string(network.getCriticNeuronConfig().ETA) +
                    "\nTAU_K: " + std::to_string(network.getCriticNeuronConfig().TAU_K) +
                    "\nNU_K: " + std::to_string(network.getCriticNeuronConfig().NU_K) + "\n";
        }
    }

    sim.stopSimulation();
    network.save(1, "Simulation");
    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");

    std::string networkPath;
    if (argc > 1) {
        networkPath = static_cast<std::string>(argv[1]) + "/configs/network_config.json";
    } else {
        networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    }

    launchLearning(networkPath);
}