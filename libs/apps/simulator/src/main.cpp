//
// Created by Thomas on 28/06/2021.
//

#include <ros/ros.h>
#include "simulator/Launcher.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuvisysRos");

    std::string type;
    std::string m_networkPath;
    if (argc > 1) {
        type = argv[1];
    }

    auto networkPath = std::string("/home/thomas/Networks/simulation/rl/tracking_task/3actions/network_learning/");
    launchNetworkSimulation(networkPath, true, 10);

//    launchValidationMultiWeights();

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
