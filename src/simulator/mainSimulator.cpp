//
// Created by thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

int launchLearningSimulation(std::string &networkPath, double simTime) {
    SimulationInterface sim;
    sim.enableSyncMode(true);
    sim.startSimulation();

    NetworkHandle network(networkPath, sim.getSimulationTime());

    int action;
    while (ros::ok() && sim.getSimulationTime() < simTime) {
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

int launchSimulation(double simTime) {
    SimulationInterface sim(false, true);
    sim.enableSyncMode(true);
    sim.startSimulation();

    while (ros::ok() && sim.getSimulationTime() < simTime) {
        sim.triggerNextTimeStep();
        while(!sim.simStepDone()) {
            ros::spinOnce();
        }

        sim.update();
    }
    sim.stopSimulation();
    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");

    launchSimulation(3.5);

//    std::string type = argv[1];
//    std::string type = "real";
//
//    std::string networkPath;
//    if (type == "multi") {
//        for (const auto &entry : std::filesystem::directory_iterator(argv[1])) {
//            networkPath = static_cast<std::string>(entry.path()) + "/configs/network_config.json";
//            std::cout << networkPath << std::endl;
//            launchLearningSimulation(networkPath);
//        }
//    } else if (type == "simulation") {
//        networkPath = static_cast<std::string>(argv[2]) + "/configs/network_config.json";
//        launchLearningSimulation(networkPath);
//    }  else if (type == "real") {
//        networkPath = static_cast<std::string>(argv[2]) + "/configs/network_config.json";
//        launchLearningSimulation(networkPath);
//    } else {
//        networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
//        launchLearningSimulation(networkPath);
//    }
}