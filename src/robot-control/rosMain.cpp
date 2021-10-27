//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

inline void updateActor(SpikingNetwork &spinet, long timestamp, size_t actor) {
    auto neuron = spinet.getNeuron(actor, spinet.getNetworkStructure().size() - 1);
    neuron.get().spike(timestamp);

    auto meanDValues = 50 * Util::secondOrderNumericalDifferentiationMean(spinet.getSaveData()["value"].end() - 50, spinet.getSaveData()["value"].end());

    neuron.get().setNeuromodulator(meanDValues);
    neuron.get().weightUpdate(); // TODO: what about the eligibility traces (previous action) ?
    spinet.normalizeActions();
//    std::this_thread::sleep_for(2s);
}

inline double getConvergence(SpikingNetwork &spinet, long time) {
    double mean = 0;
    if (spinet.getSaveData()["reward"].size() > time) {
        for (auto it = spinet.getSaveData()["reward"].end(); it != spinet.getSaveData()["reward"].end() - time && it != spinet.getSaveData()["reward"].begin(); --it) {
            mean += *it;
        }
        mean /= static_cast<double>(time);
        spinet.getSaveData()["score"].push_back(mean);
        return mean;
    }
    return 0;
}

int launchLearning(std::string &networkPath) {
    SpikingNetwork spinet(networkPath);
    spinet.loadNetwork();

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

        if (!sim.getLeftEvents().empty()) {
            spinet.updateTDError(sim.getLeftEvents().back().timestamp(), true);
        }

        sim.update();
        spinet.runEvents(sim.getLeftEvents(), sim.getReward());

        if (sim.getSimulationTime() - actionTime > static_cast<double>(spinet.getNetworkConfig().getActionRate()) / Conf::E6) {
            actionTime = sim.getSimulationTime();
            if (actor != -1) {
                updateActor(spinet, sim.getLeftEvents().back().timestamp(), actor);
            }
            sim.motorAction(spinet.resolveMotor(), spinet.getNetworkConfig().getExplorationFactor(), actor);
        }

        if (sim.getSimulationTime() - consoleTime > 10) {
            consoleTime = sim.getSimulationTime();
            spinet.learningDecay(iteration);
            ++iteration;
            std::string msg = "Average reward: " + std::to_string(getConvergence(spinet, 1000)) +
                              "\nExploration factor: " + std::to_string(spinet.getNetworkConfig().getExplorationFactor()) +
                              "\nAction rate: " + std::to_string(spinet.getNetworkConfig().getActionRate()) +
                              "\nETA: " + std::to_string(spinet.getCriticNeuronConfig().ETA) +
                              "\nTAU_K: " + std::to_string(spinet.getCriticNeuronConfig().TAU_K) +
                              "\nNU_K: " + std::to_string(spinet.getCriticNeuronConfig().NU_K) + "\n";
        }
    }

    sim.stopSimulation();
    spinet.saveNetwork(1, "Simulation");
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