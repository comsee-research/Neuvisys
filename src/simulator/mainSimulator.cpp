//
// Created by Thomas on 28/06/2021.
//

#include "SimulationInterface.hpp"

int launchLearningSimulation(std::string &networkPath, double simTime) {
    NetworkHandle network(networkPath);

    SimulationInterface sim;
    sim.enableSyncMode(true);
    sim.startSimulation();

    int action;
    while (ros::ok() && sim.getSimulationTime() < simTime) {
        sim.triggerNextTimeStep();
        while(!sim.simStepDone()) {
            ros::spinOnce();
        }

        sim.update();
        std::string msg;
        if (!sim.getLeftEvents().empty()) {
//            network.transmitReward(sim.getReward());
            for (auto const &event : sim.getLeftEvents()) {
                network.transmitEvent(event);
            }
            action = network.learningLoop(sim.getLeftEvents().back().timestamp(), sim.getSimulationTime(), 0, msg);

            if (action != -1) {
                sim.activateMotors(action);
            }
        }
    }

    sim.stopSimulation();
    network.save("Simulation", 1);
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

//    std::string type = "none";//argv[1];
//    std::string m_networkPath;
//
//    if (type == "multi") {
//        for (const auto &entry : std::filesystem::directory_iterator(argv[1])) {
//            m_networkPath = static_cast<std::string>(entry.path()) + "/configs/network_config.json";
//            std::cout << m_networkPath << std::endl;
//            launchLearningSimulation(m_networkPath, 10);
//        }
//    } else {
//        m_networkPath = "/home/thomas/Desktop/network_experiment/configs/network_config.json";
//        std::cout << m_networkPath << std::endl;
//        launchLearningSimulation(m_networkPath, 10);
//    }
}