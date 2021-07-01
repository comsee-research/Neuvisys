//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    std::string networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    SpikingNetwork spinet(networkPath);
    SimulationInterface sim(spinet);

//    ros::spin();

    auto start = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::now();
    while(std::chrono::duration_cast<std::chrono::milliseconds>(time - start).count() < 60000) {
        time = std::chrono::system_clock::now();
        ros::spinOnce();
        sim.update();
    }

    return 0;
}