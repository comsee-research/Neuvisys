//
// Created by Thomas on 14/04/2021.
//

#include "NeuvisysthreadSimulation.h"

NeuvisysThreadSimulation::NeuvisysThreadSimulation(int argc, char **argv, QObject *parent) : NeuvisysThread(argc, argv, parent) {
}

bool NeuvisysThreadSimulation::init() {
    ros::init(m_initArgc, m_initArgv, "neuvisysRos");
    if (!ros::master::check()) {
        return false;
    }
    ros::start();
    ros::Time::init();
    return true;
}

void NeuvisysThreadSimulation::run() {
    if (m_mode == 3) {
        auto network = NetworkHandle();
        m_leftEventDisplay = cv::Mat::zeros(260, 346, CV_8UC3);
        m_rightEventDisplay = cv::Mat::zeros(260, 346, CV_8UC3);
        if (m_events.toStdString().empty()) {
            readEventsSimulation();
//            readEventsRealTime();
        } else {
            network.setEventPath(m_events.toStdString());
            readEventsFile(network);
        }
    } else {
        auto network = NetworkHandle(m_networkPath.toStdString());
        m_leftEventDisplay = cv::Mat::zeros(network.getNetworkConfig().getVfHeight(), network.getNetworkConfig().getVfWidth(), CV_8UC3);
        m_rightEventDisplay = cv::Mat::zeros(network.getNetworkConfig().getVfHeight(), network.getNetworkConfig().getVfWidth(), CV_8UC3);

        emit networkConfiguration(network.getNetworkConfig().getSharingType(),
                                  network.getNetworkConfig().getLayerPatches()[0],
                                  network.getNetworkConfig().getLayerSizes()[0],
                                  network.getNetworkConfig().getNeuronSizes()[0]);
        emit networkCreation(network.getNetworkConfig().getNbCameras(), network.getNetworkConfig().getNeuron1Synapses(),
                             network.getNetworkStructure(), network.getNetworkConfig().getVfWidth(), network.getNetworkConfig().getVfHeight());
        m_motorDisplay = std::vector<bool>(2, false);

        if (m_mode == 0) {
            network.setEventPath(m_events.toStdString());
            m_endTime = static_cast<double>(m_nbPass) * (network.getLastTimestamp() - network.getFirstTimestamp());
            launchNetwork(network);
        } else if (m_mode == 1) {
            launchReal(network);
        } else if (m_mode == 2) {
            launchSimulation(network);
        }
    }
    emit networkDestruction();
    quit();
}

void NeuvisysThreadSimulation::readEventsSimulation() {
    SimulationInterface sim(vector<pair<uint64_t, float>>{}, true, true);
    sim.enableSyncMode(true);
    sim.startSimulation();

    while (!m_stop) {
        sim.triggerNextTimeStep();
        while (!sim.simStepDone() && !m_stop) {
            ros::spinOnce();
        }
        sim.update();
    }

    sim.stopSimulation();
}

void NeuvisysThreadSimulation::launchSimulation(NetworkHandle &network) {
    SimulationInterface sim(network.getRLConfig().getActionMapping());
    sim.enableSyncMode(true);
    sim.startSimulation();

    while (!m_stop) {
        sim.triggerNextTimeStep();
        while (!sim.simStepDone() && !m_stop) {
            ros::spinOnce();
        }
        sim.update();

        if (!network.getRLConfig().getIntrinsicReward()) {
            network.transmitReward(sim.getReward());
        }
        eventLoop(network, sim.getLeftEvents(), sim.getSimulationTime() * E6);
        if (m_action != -1) {
            sim.activateMotors(m_action);
            m_motorDisplay[m_action] = true;
        }

        if (sim.getSimulationTime() > 300) {
            m_stop = true;
        }
    }
    sim.stopSimulation();
    network.save("Simulation", 1);
}

NeuvisysThreadSimulation::~NeuvisysThreadSimulation() {
    if (ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }
}

