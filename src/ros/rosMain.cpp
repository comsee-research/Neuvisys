//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");
    std::string networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    SpikingNetwork spinet(networkPath);

    spinet.addLayer("SimpleCell", spinet.getNetworkConfig().SharingType, true, spinet.getNetworkConfig().layerPatches[0], spinet.getNetworkConfig().layerSizes[0], spinet.getNetworkConfig().neuronSizes[0], 0);
    spinet.addLayer("ComplexCell", "none", true, spinet.getNetworkConfig().layerPatches[1], spinet.getNetworkConfig().layerSizes[1], spinet.getNetworkConfig().neuronSizes[1], 0);
    spinet.addLayer("CriticCell", "none", false, spinet.getNetworkConfig().layerPatches[2], spinet.getNetworkConfig().layerSizes[2], spinet.getNetworkConfig().neuronSizes[2], 1);
    spinet.addLayer("ActorCell", "none", false, spinet.getNetworkConfig().layerPatches[3], spinet.getNetworkConfig().layerSizes[3], spinet.getNetworkConfig().neuronSizes[3], 1);
    spinet.loadNetwork();

    SimulationInterface sim(1. / 150);

    auto start = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::now();

    std::chrono::time_point<std::chrono::system_clock> motorTime;
    motorTime = std::chrono::high_resolution_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(time - start).count() < 60 * 1000) {
        ros::spinOnce();

        if (sim.hasReceivedLeftImage()) {
            time = std::chrono::system_clock::now();

            sim.update();

            spinet.runEvents(sim.getLeftEvents(), sim.getReward());

            std::chrono::duration<double> frameElapsed = std::chrono::high_resolution_clock::now() - motorTime;
            if (1000000 * frameElapsed.count() > static_cast<double>(100000)) {
                motorTime = std::chrono::high_resolution_clock::now();

//                int selectedMotor;
//                auto exploration = sim.motorAction(spinet.resolveMotor(), 0, selectedMotor);
//                if (!sim.getLeftEvents().empty() && exploration) {
//                    auto neuron = spinet.getNeuron(selectedMotor, spinet.getNetworkStructure().size()-1);
//                    neuron.spike(sim.getLeftEvents().back().timestamp());
//                    neuron.setNeuromodulator(spinet.critic(neuron.getIndex()));
//                    neuron.weightUpdate();
//                    neuron.resetSpike();
//                }
            }

            sim.resetLeft();
        }
    }

    spinet.saveNetwork(1, "Simulation");
    std::cout << "Finished" << std::endl;
    return 0;
}
