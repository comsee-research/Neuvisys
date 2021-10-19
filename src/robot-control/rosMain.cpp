//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

inline void updateActor(SpikingNetwork &spinet, long timestamp, size_t actor) {
    auto neuron = spinet.getNeuron(actor, spinet.getNetworkStructure().size() - 1);
    neuron.get().spike(timestamp);

    auto first = spinet.getListValue().end() - 20;
    auto last = spinet.getListValue().end();
    auto meanDValues = 50 * Util::secondOrderNumericalDifferentiationMean(first, last);

    neuron.get().setNeuromodulator(meanDValues);
    neuron.get().weightUpdate(); // TODO: what about the eligibility traces (previous action) ?
//    std::this_thread::sleep_for(2s);
}

inline double getConvergence(SpikingNetwork &spinet) {
    double mean = 0;
    for (auto it = spinet.getRewards().end(); it != spinet.getRewards().end() - 1000 && it != spinet.getRewards().begin(); --it) {
        mean += *it;
    }
    mean /= 1000;
    return mean;
}

int launchLearning(std::string &networkPath) {
    SpikingNetwork spinet(networkPath);
    spinet.loadNetwork();

    SimulationInterface sim(1. / 150);
    sim.enableSyncMode(true);
    sim.startSimulation();

    std::vector<size_t> vecEvents;

    double actionTime = 0, displayTime = 0;
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

        if (sim.getSimulationTime() - displayTime > 10) {
            displayTime = sim.getSimulationTime();

            spinet.learningDecay(iteration);
            ++iteration;
            std::cout << "Average reward: " << getConvergence(spinet) << "\nExploration factor: " << spinet.getNetworkConfig().getExplorationFactor() <<
            "\n" << std::endl;
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