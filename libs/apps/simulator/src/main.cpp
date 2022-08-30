//
// Created by Thomas on 28/06/2021.
//

#include <ros/ros.h>

#include <simulator/SimulationInterface.hpp>
#include <network/NetworkHandle.hpp>

void launchLearningSimulation(std::string &networkPath, double simTime) {
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
            network.updateNeurons(static_cast<size_t>(sim.getSimulationTime() * E6));

            if (network.getRLConfig().getRLTraining()) {
                action = network.learningLoop(sim.getLeftEvents().back().timestamp(), sim.getSimulationTime() * E6, sim.getLeftEvents().size(), msg);
            }
            if (action != -1) {
//                sim.activateMotors(action);
            }
        }
    }

    sim.stopSimulation();
    network.save("Simulation", 1);
}

int launchSimulation(double simTime) {
    std::mt19937 generator(static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<int> distr(0, 2);
    std::vector<std::pair<uint64_t, float>> actionMapping({{1, 5}, {1, 0}, {1, -5}});

    SimulationInterface sim(actionMapping, true, false);
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

void simulationValidation() {
    std::string networkPath = "/home/thomas/Networks/simulation/rl/orientation_task/article/validation/";
    std::string srcPath = "/home/thomas/Networks/simulation/rl/orientation_task/article/intermediate/intermediate_";
    std::string destPath = networkPath + "weights/";

    std::ifstream src;
    std::ofstream dst;
    for (size_t i = 0; i < 50; ++i) {
        for (size_t j = 0; j < 100; ++j) {
            src = std::ifstream(srcPath + std::to_string(i) + "/2/" + std::to_string(j) + "_0.npy", std::ios::binary);
            dst = std::ofstream(destPath + "2/" + std::to_string(j) + "_0.npy", std::ios::binary);
            dst << src.rdbuf();

            src = std::ifstream(srcPath + std::to_string(i) + "/2/" + std::to_string(j) + "_1.npy", std::ios::binary);
            dst = std::ofstream(destPath + "2/" + std::to_string(j) + "_1.npy", std::ios::binary);
            dst << src.rdbuf();

            src = std::ifstream(srcPath + std::to_string(i) + "/3/" + std::to_string(j) + "_0.npy", std::ios::binary);
            dst = std::ofstream(destPath + "3/" + std::to_string(j) + "_0.npy", std::ios::binary);
            dst << src.rdbuf();

            src = std::ifstream(srcPath + std::to_string(i) + "/3/" + std::to_string(j) + "_1.npy", std::ios::binary);
            dst = std::ofstream(destPath + "3/" + std::to_string(j) + "_1.npy", std::ios::binary);
            dst << src.rdbuf();
        }
        launchLearningSimulation(networkPath, 2.6); // tracking: 6, orientation: 1.3
        const auto copyOptions = fs::copy_options::recursive;
        std::filesystem::copy(networkPath, "/home/thomas/Networks/simulation/rl/orientation_task/article/save/" + std::to_string(i), copyOptions);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");

    std::string type;
    std::string m_networkPath;
    if (argc > 1) {
        type = argv[1];
    }

//    launchSimulation(10);
    simulationValidation();

//    if (type == "multi") {
//        for (const auto &entry : std::filesystem::directory_iterator(argv[1])) {
//            m_networkPath = static_cast<std::string>(entry.path()) + "/configs/network_config.json";
//            std::cout << m_networkPath << std::endl;
//            launchLearningSimulation(m_networkPath, 10);
//        }
//    } else if (type == "single") {
//        m_networkPath = argv[2];
//        launchLearningSimulation(m_networkPath, 2.6);
//    } else {
//        m_networkPath = "/home/thomas/Networks/simulation/rl/tracking_task/article/validation/";
//        launchLearningSimulation(m_networkPath, 2.6);
//    }
//    return 0;
}
