//
// Created by thomas on 28/06/2021.
//

#include <motor-control/StepMotor.hpp>
#include "SimulationInterface.hpp"
#include "../dv-modules/DavisHandle.hpp"

static atomic_bool globalShutdown(false);

static void globalShutdownSignalHandler(int signal) {
    // Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
    if (signal == SIGTERM || signal == SIGINT) {
        globalShutdown.store(true);
    }
}

static void usbShutdownHandler(void *ptr) {
    (void) (ptr); // UNUSED.

    globalShutdown.store(true);
}

int launchLearningSimulation(std::string &networkPath) {
    SimulationInterface sim;
    sim.enableSyncMode(true);
    sim.startSimulation();

    NetworkHandle network(networkPath, sim.getSimulationTime());

    int action = 0;
    std::string msg;
    while (ros::ok() && sim.getSimulationTime() < 300) {
        sim.triggerNextTimeStep();
        while(!sim.simStepDone()) {
            ros::spinOnce();
        }

        sim.update();
        if (!sim.getLeftEvents().empty()) {
            network.transmitReward(sim.getReward());
            network.transmitEvents(sim.getLeftEvents());
            action = network.learningLoop(sim.getLeftEvents().back().timestamp(), sim.getSimulationTime(), msg);

            if (action != -1) {
                sim.activateMotors(action);
            }
            std::cout << msg << std::endl;
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
        launchLearningSimulation(networkPath);
    }  else if (type == "real") {
//        networkPath = static_cast<std::string>(argv[2]) + "/configs/network_config.json";
        networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    } else {
        networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
        launchLearningSimulation(networkPath);
    }
}