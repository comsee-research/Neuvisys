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

    spinet.addLayer("SimpleCell", spinet.getNetworkConfig().getSharingType(), true,
                    spinet.getNetworkConfig().getLayerPatches()[0], spinet.getNetworkConfig().getLayerSizes()[0],
                    spinet.getNetworkConfig().getNeuronSizes()[0], spinet.getNetworkConfig().getNeuronOverlap()[0], 0);
    spinet.addLayer("ComplexCell", "none", true, spinet.getNetworkConfig().getLayerPatches()[1],
                    spinet.getNetworkConfig().getLayerSizes()[1], spinet.getNetworkConfig().getNeuronSizes()[1],
                    spinet.getNetworkConfig().getNeuronOverlap()[1], 0);
    spinet.addLayer("CriticCell", "none", false, spinet.getNetworkConfig().getLayerPatches()[2],
                    spinet.getNetworkConfig().getLayerSizes()[2], spinet.getNetworkConfig().getNeuronSizes()[2],
                    spinet.getNetworkConfig().getNeuronOverlap()[2], 1);
    spinet.addLayer("ActorCell", "none", true, spinet.getNetworkConfig().getLayerPatches()[3],
                    spinet.getNetworkConfig().getLayerSizes()[3], spinet.getNetworkConfig().getNeuronSizes()[3],
                    spinet.getNetworkConfig().getNeuronOverlap()[3], 1);
    spinet.loadNetwork();

    SimulationInterface sim(1. / 150);
    sim.enableSyncMode(true);
    sim.startSimulation();

    double simTime = 0;
    int actor = 0;
    while (ros::ok() && sim.getSimulationTime() < 50) {
        sim.triggerNextTimeStep();
        while(!sim.simStepDone()) {
            ros::spinOnce();
        }

        sim.update();
        spinet.runEvents(sim.getLeftEvents(), sim.getReward());

        if (sim.getSimulationTime() - simTime > 0.1) {
            simTime = sim.getSimulationTime();
            if (actor != -1) {
                auto neuron = spinet.getNeuron(actor, spinet.getNetworkStructure().size()-1);
                neuron.get().spike(sim.getLeftEvents().back().timestamp());
                neuron.get().setNeuromodulator(spinet.updateTDError(sim.getLeftEvents().back().timestamp()));
                neuron.get().weightUpdate();
            }
            sim.motorAction(spinet.resolveMotor(), 0, actor);
        }
    }

    sim.stopSimulation();
    spinet.saveNetwork(1, "Simulation");
    std::cout << "Finished" << std::endl;
    return 0;
}
