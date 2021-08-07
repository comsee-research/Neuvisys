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

    std::chrono::time_point<std::chrono::system_clock> motorTime;
    motorTime = std::chrono::high_resolution_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(time - start).count() < 180 * 1000) {
        ros::spinOnce();

        if (sim.hasReceivedLeftImage()) {
            time = std::chrono::system_clock::now();

            sim.update();

            spinet.runEvents(sim.getLeftEvents(), sim.getReward());

            std::chrono::duration<double> frameElapsed = std::chrono::high_resolution_clock::now() - motorTime;
            if (1000000 * frameElapsed.count() > static_cast<double>(100000)) {
                motorTime = std::chrono::high_resolution_clock::now();

                auto selectedMotor = sim.motorAction(spinet.resolveMotor());
            }

            sim.resetLeft();
        }
    }

    spinet.saveNetwork(1, "Simulation");
    std::cout << "Finished" << std::endl;
    return 0;
}
