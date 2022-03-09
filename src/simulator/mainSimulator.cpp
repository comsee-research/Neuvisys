//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

int launchLearningSimulation(std::string &networkPath) {
    SimulationInterface sim;
    sim.enableSyncMode(true);
    sim.startSimulation();

    NetworkHandle network(networkPath, sim.getSimulationTime());

    int action;
    while (ros::ok() && sim.getSimulationTime() < 300) {
        sim.triggerNextTimeStep();
        while(!sim.simStepDone()) {
            ros::spinOnce();
        }

        sim.update();
        std::string msg;
        if (!sim.getLeftEvents().empty()) {
            network.transmitReward(sim.getReward());
            network.transmitEvents(sim.getLeftEvents());
            action = network.learningLoop(sim.getLeftEvents().back().timestamp(), sim.getSimulationTime(), msg);

            if (action != -1) {
                sim.activateMotors(action);
            }
        }
    }

    sim.stopSimulation();
    network.save(1, "Simulation");
    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");

//    std::string type = argv[1];
    std::string type = "real";

    std::string networkPath;
    if (type == "multi") {
        for (const auto &entry : std::filesystem::directory_iterator(argv[1])) {
            networkPath = static_cast<std::string>(entry.path()) + "/configs/network_config.json";
            std::cout << networkPath << std::endl;
            launchLearningSimulation(networkPath);
        }
    } else if (type == "simulation") {
        networkPath = static_cast<std::string>(argv[2]) + "/configs/network_config.json";
    }  else if (type == "real") {
//        networkPath = static_cast<std::string>(argv[2]) + "/configs/network_config.json";
        networkPath = "/home/thomas/Bureau/network/configs/network_config.json";
    } else {
        networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    }
    launchLearningSimulation(networkPath);
}