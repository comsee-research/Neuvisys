//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");
    std::string networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    SpikingNetwork spinet(networkPath);
    SimulationInterface sim;

    auto start = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(time - start).count() < 180 * 1000) {
        time = std::chrono::system_clock::now();
        ros::spinOnce();

        if (sim.hasReceivedLeftImage()) {
            auto dt = sim.update();
            spinet.runEvents(sim.getLeftEvents(), sim.getReward());

            auto activations = spinet.getMotorActivation();
            sim.activateMotors(activations);
            spinet.resetMotorActivation();
            sim.resetLeft();
        }
    }

    spinet.saveNetwork(1, "Simulation");
    std::cout << "Finished" << std::endl;
    return 0;
}
