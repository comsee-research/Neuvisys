//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");
    std::string networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    SpikingNetwork spinet(networkPath);
    SimulationInterface sim(1. / 150);

    auto start = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(time - start).count() < 180 * 1000) {
        ros::spinOnce();

        if (sim.hasReceivedLeftImage()) {
            time = std::chrono::system_clock::now();

            auto selectedMotor = sim.update();

            spinet.runEvents(sim.getLeftEvents(), sim.getReward());

            selectedMotor = sim.motorAction(spinet.getMotorActivation());

            spinet.resetMotorActivation();
            sim.resetLeft();
        }
    }

    spinet.saveNetwork(1, "Simulation");
    std::cout << "Finished" << std::endl;
    return 0;
}
