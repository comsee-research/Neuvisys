//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp""

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");

    std::string networkPath;
    if (argc > 1) {
        networkPath = static_cast<std::string>(argv[1]) + "/configs/network_config.json";
    } else {
        networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    }
    SpikingNetwork spinet(networkPath);
    spinet.loadNetwork();

    SimulationInterface sim(1. / 150);
    sim.enableSyncMode(true);
    sim.startSimulation();

    double simTime = 0;
    int actor = 0;
    while (ros::ok() && sim.getSimulationTime() < 1) {
        sim.triggerNextTimeStep();
        while(!sim.simStepDone()) {
            ros::spinOnce();
        }

        if (!sim.getLeftEvents().empty()) {
            spinet.updateTDError(sim.getLeftEvents().back().timestamp(), true);
        }

        sim.update();
        spinet.runEvents(sim.getLeftEvents(), sim.getReward());

//        if (sim.getSimulationTime() - simTime > 0.1) {
//            simTime = sim.getSimulationTime();
//            if (actor != -1) {
//                auto neuron = spinet.getNeuron(actor, spinet.getNetworkStructure().size() - 1);
//                neuron.get().spike(sim.getLeftEvents().back().timestamp());
//                size_t count = 0;
//                double td = 0;
//                for (auto rit = spinet.getListTDError().rbegin(); rit != spinet.getListTDError().rend(); ++rit) {
//                    if (count > 9) {
//                        break;
//                    }
//                    td += *rit;
//                    ++count;
//                }
//                neuron.get().setNeuromodulator(td / 10);
//                neuron.get().weightUpdate();
//            }
//            sim.motorAction(spinet.resolveMotor(), 0, actor);
//        }
    }

    sim.stopSimulation();
    spinet.saveNetwork(1, "Simulation");
    return 0;
}
