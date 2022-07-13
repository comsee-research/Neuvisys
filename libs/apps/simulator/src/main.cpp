//
// Created by Thomas on 28/06/2021.
//

#include <ros/ros.h>

#include <simulator/SimulationInterface.hpp>
#include <network/NetworkHandle.hpp>

int launchLearningSimulation(std::string &networkPath, double simTime) {
    NetworkHandle network(networkPath);

    SimulationInterface sim(network.getRLConfig().getActionMapping());
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

        if (!network.getRLConfig().getIntrinsicReward()) {
            network.transmitReward(sim.getReward());
        }
        if (!sim.getLeftEvents().empty()) {
            for (auto const &event: sim.getLeftEvents()) {
                network.transmitEvent(event);
            }
            network.updateNeurons(static_cast<size_t>(sim.getSimulationTime()));

            if (network.getRLConfig().getRLTraining()) {
                action = network.learningLoop(sim.getLeftEvents().back().timestamp(), sim.getSimulationTime(), 0, msg);
            }
            if (action != -1) {
                sim.activateMotors(action);
            }
        }
    }

    sim.stopSimulation();
//    network.save("Simulation", 1);
    return 0;
}

int launchSimulation(double simTime) {
    std::mt19937 generator(static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<int> distr(0, 2);
    std::vector<std::pair<uint64_t, float>> actionMapping({{1, 5}, {1, 0}, {1, -5}});

    SimulationInterface sim(actionMapping, false, false);
    sim.enableSyncMode(true);
    sim.startSimulation();

    while (ros::ok() && sim.getSimulationTime() < simTime) {
        sim.triggerNextTimeStep();
        while(!sim.simStepDone()) {
            ros::spinOnce();
        }

        sim.update();

        if (static_cast<int>(sim.getSimulationTime()) % 2 == 0) {
            sim.activateMotors(distr(generator));
        }
    }
    sim.stopSimulation();
    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");

    std::string type = "none";//argv[1];
    std::string m_networkPath;

//    launchSimulation(10);

    if (type == "multi") {
        for (const auto &entry : std::filesystem::directory_iterator(argv[1])) {
            m_networkPath = static_cast<std::string>(entry.path()) + "/configs/network_config.json";
            std::cout << m_networkPath << std::endl;
            launchLearningSimulation(m_networkPath, 10);
        }
    } else {
        m_networkPath = "/home/thomas/Networks/simulation/rl/orientation_task/skip_connections/network_learning/";
        launchLearningSimulation(m_networkPath, 10);
    }
    return 0;
}
