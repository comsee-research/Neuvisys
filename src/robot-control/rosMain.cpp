//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");

    std::string networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    SpikingNetwork spinet(networkPath);

    spinet.addLayer("SimpleCell", spinet.getNetworkConfig().getSharingType(), true, spinet.getNetworkConfig().getLayerPatches()[0], spinet.getNetworkConfig().getLayerSizes()[0], spinet.getNetworkConfig().getNeuronSizes()[0], 0);
    spinet.addLayer("ComplexCell", "none", true, spinet.getNetworkConfig().getLayerPatches()[1], spinet.getNetworkConfig().getLayerSizes()[1], spinet.getNetworkConfig().getNeuronSizes()[1], 0);
    spinet.addLayer("CriticCell", "none", false, spinet.getNetworkConfig().getLayerPatches()[2], spinet.getNetworkConfig().getLayerSizes()[2], spinet.getNetworkConfig().getNeuronSizes()[2], 1);
    spinet.addLayer("ActorCell", "none", false, spinet.getNetworkConfig().getLayerPatches()[3], spinet.getNetworkConfig().getLayerSizes()[3], spinet.getNetworkConfig().getNeuronSizes()[3], 1);
    spinet.loadNetwork();

    SimulationInterface sim(1. / 150);
    sim.enableSyncMode(true);
    sim.startSimulation();

    while (ros::ok() && sim.getSimulationTime() < 5) {
        sim.triggerNextTimeStep();
        ros::spinOnce();

        if (sim.hasReceivedLeftImage()) {
            sim.update();
            spinet.runEvents(sim.getLeftEvents(), sim.getReward());
            sim.resetLeft();
        }
    }

    sim.stopSimulation();
    spinet.saveNetwork(1, "Simulation");
    std::cout << "Finished" << std::endl;
    return 0;
}
