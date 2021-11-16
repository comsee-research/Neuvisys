#include "neuvisysthread.h"
#include <utility>
#include <thread>
#include <chrono>

using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

NeuvisysThread::NeuvisysThread(int argc, char **argv, QObject *parent) : QThread(parent), m_initArgc(argc),
                                                                         m_initArgv(argv) {
    m_iterations = 0;
    m_nbPass = 0;
}

bool NeuvisysThread::init() {
    ros::init(m_initArgc, m_initArgv, "neuvisysRos");
    if (!ros::master::check()) {
        return false;
    }
    ros::start();
    ros::Time::init();
    return true;
}

void NeuvisysThread::render(QString networkPath, QString events, size_t nbPass, bool realtime) {
    m_networkPath = std::move(networkPath);
    m_events = std::move(events);
    m_nbPass = nbPass;
    m_iterations = 0;
    m_realtime = realtime;
    start(HighPriority);
}

void NeuvisysThread::run() {
    auto network = NetworkHandle(m_networkPath.toStdString());

    emit networkConfiguration(network.getNetworkConfig().getSharingType(),
                              network.getNetworkConfig().getLayerPatches()[0],
                              network.getNetworkConfig().getLayerSizes()[0],
                              network.getNetworkConfig().getNeuronSizes()[0]);
    emit networkCreation(network.getNetworkConfig().getNbCameras(), network.getNetworkConfig().getNeuron1Synapses(),
                         network.getNetworkStructure());
    m_leftEventDisplay = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);
    m_rightEventDisplay = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);
    m_motorDisplay = std::vector<bool>(2, false);

    if (m_realtime) {
        rosPass(network);
    } else {
        multiplePass(network);
    }
    quit();
}

void NeuvisysThread::multiplePass(NetworkHandle &network) {
    auto eventPacket = std::vector<Event>();
    if (network.getNetworkConfig().getNbCameras() == 1) {
        eventPacket = NetworkHandle::mono(m_events.toStdString(), m_nbPass);
    } else if (network.getNetworkConfig().getNbCameras() == 2) {
        eventPacket = NetworkHandle::stereo(m_events.toStdString(), m_nbPass);
    }
    emit displayProgress(100, 0, 0, 0, 0, 0, 0);

    network.transmitEvents(eventPacket);

//    for (auto event: eventPacket) {
//        addEventToDisplay(event);
//        network.transmitEvents(event);
//
//        if () { event clock (30ms)
//            display(network, eventPacket.size());
//        }
//
//        if () { event clock (10ms)
//            network.trackNeuron(event.timestamp(), m_id, m_layer);
//        }
//    }

    network.save(m_nbPass, m_events.toStdString());
    emit networkDestruction();
}

void NeuvisysThread::rosPass(NetworkHandle &network) {
    SimulationInterface sim(1. / 150);
    sim.enableSyncMode(true);
    sim.startSimulation();

    m_simTimeStep = static_cast<size_t>(sim.getSimulationTimeStep());

    int actor = 0;
    double actionTime = 0, displayTime = 0, updateTime = 0, trackTime = 0, consoleTime = 0;
    size_t iteration = 0;
    while (!m_stop) {
        sim.triggerNextTimeStep();
        while (!sim.simStepDone() && !m_stop) {
            ros::spinOnce();
        }

        sim.update();
        if (!sim.getLeftEvents().empty()) {
            network.transmitReward(sim.getReward());
            network.transmitEvents(sim.getLeftEvents());
            m_eventRate += static_cast<double>(sim.getLeftEvents().size());

            if (sim.getSimulationTime() - updateTime > static_cast<double>(UPDATE_INTERVAL) / E6) {
                updateTime = sim.getSimulationTime();
                network.updateNeuronStates(UPDATE_INTERVAL);
            }

            if (sim.getSimulationTime() - actionTime > static_cast<double>(network.getNetworkConfig().getActionRate()) / E6) {
                actionTime = sim.getSimulationTime();
//                if (actor != -1) {
//                    network.updateActor(sim.getLeftEvents().back().timestamp(), actor);
//                }
//                sim.motorAction(network.resolveMotor(), network.getNetworkConfig().getExplorationFactor(), actor);

                if (actor != -1) {
                    m_motorDisplay[actor] = true;
                }
            }

            if (sim.getSimulationTime() - displayTime > m_displayRate / E6) {
                displayTime = sim.getSimulationTime();
                display(network, 0, displayTime);
            }

            if (sim.getSimulationTime() - trackTime > m_trackRate / E6) {
                trackTime = sim.getSimulationTime();
                if (!sim.getLeftEvents().empty()) {
                    network.trackNeuron(sim.getLeftEvents().back().timestamp(), m_id, m_layer);
                }
            }

            if (sim.getSimulationTime() - consoleTime > SCORE_INTERVAL) {
                consoleTime = sim.getSimulationTime();
                network.learningDecay(iteration);
                ++iteration;
                std::string msg = "\n\nAverage reward: " + std::to_string(network.getScore(SCORE_INTERVAL * E3 / DT)) +
                                  "\nExploration factor: " + std::to_string(network.getNetworkConfig().getExplorationFactor()) +
                                  "\nAction rate: " + std::to_string(network.getNetworkConfig().getActionRate());
                emit consoleMessage(msg);
            }
        }
    }
    sim.stopSimulation();
    network.save(1, "Simulation");
    emit networkDestruction();
}

inline void NeuvisysThread::addEventToDisplay(const Event &event) {
    if (event.polarity() == 0) {
        ++m_off_count;
    } else {
        ++m_on_count;
    }
    if (event.camera() == 0) {
        if (m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[1] == 0 &&
            m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2] == 0) {
            m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2 - event.polarity()] = 255;
        }
    } else {
        if (m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[1] == 0 &&
            m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2] == 0) {
            m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2 - event.polarity()] = 255;
        }
    }
}

inline void NeuvisysThread::display(NetworkHandle &network, size_t sizeArray, double time) {
    if (m_change) {
        m_change = false;
        auto sharing = "none";
        if (m_layer == 0) {
            sharing = "patch";
        }
        emit networkConfiguration(sharing, network.getNetworkConfig().getLayerPatches()[m_layer],
                                  network.getNetworkConfig().getLayerSizes()[m_layer],
                                  network.getNetworkConfig().getNeuronSizes()[m_layer]);
    }

    auto on_off_ratio = static_cast<double>(m_on_count) / static_cast<double>(m_on_count + m_off_count);
    int progress = 0;
    switch (m_currentTab) {
        case 0: // event viz
            sensingZone(network);
            emit displayEvents(m_leftEventDisplay, m_rightEventDisplay);
            emit displayAction(m_motorDisplay);
            break;
        case 1: // statistics
            m_eventRate = (E6 / m_displayRate) * m_eventRate;
            if (sizeArray != 0) {
                progress = static_cast<int>(100 * m_iterations / sizeArray);
            }
            emit displayProgress(progress, m_simTime, m_eventRate, on_off_ratio,
                                 network.getNeuron(m_id, m_layer).get().getSpikingRate(),
                                 network.getNeuron(m_id, m_layer).get().getThreshold(), 0);
            break;
        case 2: // weights
            prepareWeights(network);
            emit displayWeights(m_weightDisplay, m_layer);
            break;
        case 3: // potential
            emit displayPotential(network.getSimpleNeuronConfig().VRESET,
                                  network.getNeuron(m_id, m_layer).get().getThreshold(),
                                  network.getNeuron(m_id, m_layer).get().getTrackingPotentialTrain());
            break;
        case 4: // spiketrain
            prepareSpikes(network);
            emit displaySpike(m_spikeTrain, time * E6);
            break;
        case 5: // reward
            emit displayReward(network.getSaveData()["reward"], network.getSaveData()["value"], network.getSaveData()["valueDot"],
                               network.getSaveData()["tdError"]);
            break;
        default:
            break;
    }
    m_eventRate = 0;
    m_on_count = 0;
    m_off_count = 0;
    m_motorDisplay = std::vector<bool>(network.getNetworkStructure().back(), false);
    m_leftEventDisplay = 0;
    m_rightEventDisplay = 0;
}

inline void NeuvisysThread::sensingZone(NetworkHandle &network) {
    for (size_t i = 0; i < network.getNetworkConfig().getLayerPatches()[0][0].size(); ++i) {
        for (size_t j = 0; j < network.getNetworkConfig().getLayerPatches()[0][1].size(); ++j) {
            auto offsetXPatch = static_cast<int>(network.getNetworkConfig().getLayerPatches()[0][0][i] +
                                                 network.getNetworkConfig().getLayerSizes()[0][0] *
                                                 network.getNetworkConfig().getNeuronSizes()[0][0]);
            auto offsetYPatch = static_cast<int>(network.getNetworkConfig().getLayerPatches()[0][1][j] +
                                                 network.getNetworkConfig().getLayerSizes()[0][1] *
                                                 network.getNetworkConfig().getNeuronSizes()[0][1]);
            cv::rectangle(m_leftEventDisplay,
                          cv::Point(static_cast<int>(network.getNetworkConfig().getLayerPatches()[0][0][i]),
                                    static_cast<int>(network.getNetworkConfig().getLayerPatches()[0][1][j])),
                          cv::Point(offsetXPatch, offsetYPatch),
                          cv::Scalar(255, 0, 0));
            cv::rectangle(m_rightEventDisplay,
                          cv::Point(static_cast<int>(network.getNetworkConfig().getLayerPatches()[0][0][i]),
                                    static_cast<int>(network.getNetworkConfig().getLayerPatches()[0][1][j])),
                          cv::Point(offsetXPatch, offsetYPatch),
                          cv::Scalar(255, 0, 0));
        }
    }
}

inline void NeuvisysThread::prepareSpikes(NetworkHandle &network) {
    m_spikeTrain.clear();
    for (size_t i = 0; i < network.getNetworkStructure()[m_layer]; ++i) {
        m_spikeTrain.push_back(std::ref(network.getNeuron(i, m_layer).get().getTrackingSpikeTrain()));
    }
}

inline void NeuvisysThread::prepareWeights(NetworkHandle &network) {
    m_weightDisplay.clear();
    size_t count = 0;
    if (m_layer == 0) {
        for (size_t i = 0; i < network.getNetworkConfig().getLayerPatches()[m_layer][0].size() *
                               network.getNetworkConfig().getLayerSizes()[m_layer][0]; ++i) {
            for (size_t j = 0; j < network.getNetworkConfig().getLayerPatches()[m_layer][1].size() *
                                   network.getNetworkConfig().getLayerSizes()[m_layer][1]; ++j) {
                if (network.getNetworkConfig().getSharingType() == "none") {
                    m_weightDisplay[count] = network.getWeightNeuron(network.getLayout(0, Position(i, j, m_zcell)),
                                                                     m_layer, m_camera, m_synapse, m_zcell);
                }
                ++count;
            }
        }
        if (network.getNetworkConfig().getSharingType() == "patch") {
            count = 0;
            for (size_t wp = 0; wp < network.getNetworkConfig().getLayerPatches()[m_layer][0].size(); ++wp) {
                for (size_t hp = 0; hp < network.getNetworkConfig().getLayerPatches()[m_layer][1].size(); ++hp) {
                    for (size_t i = 0; i < network.getNetworkConfig().getLayerSizes()[m_layer][2]; ++i) {
                        m_weightDisplay[count] = network.getWeightNeuron(
                                network.getLayout(0, Position(wp * network.getNetworkConfig().getLayerSizes()[m_layer][0],
                                                              hp * network.getNetworkConfig().getLayerSizes()[m_layer][1],
                                                              i)), m_layer, m_camera,
                                m_synapse, m_zcell);
                        ++count;
                    }
                }
            }
        } else if (network.getNetworkConfig().getSharingType() == "full") {
            for (size_t i = 0; i < network.getNetworkConfig().getLayerSizes()[m_layer][2]; ++i) {
                m_weightDisplay[i] = network.getWeightNeuron(network.getLayout(0, Position(0, 0, i)), m_layer, m_camera,
                                                             m_synapse, m_zcell);
            }
        }
    } else {
        for (size_t i = 0; i < network.getNetworkConfig().getLayerSizes()[m_layer][0]; ++i) {
            m_weightDisplay[count] = network.getSummedWeightNeuron(network.getLayout(m_layer, Position(i, 0, m_zcell)), m_layer);
//            m_weightDisplay[count] = network.getWeightNeuron(network.getLayout(m_layer, Position(i, 0, m_zcell)), m_layer,
//                                                            m_camera, m_synapse, m_depth);
            ++count;
        }
    }
}

void NeuvisysThread::onTabVizChanged(size_t index) {
    m_currentTab = index;
}

void NeuvisysThread::onIndexChanged(size_t index) {
    m_id = index;
}

void NeuvisysThread::onZcellChanged(size_t zcell) {
    m_zcell = zcell;
}

void NeuvisysThread::onDepthChanged(size_t depth) {
    m_depth = depth;
}

void NeuvisysThread::onCameraChanged(size_t camera) {
    m_camera = camera;
}

void NeuvisysThread::onSynapseChanged(size_t synapse) {
    m_synapse = synapse;
}

void NeuvisysThread::onPrecisionEventChanged(size_t displayRate) {
    m_displayRate = static_cast<double>(displayRate);
}

void NeuvisysThread::onPrecisionPotentialChanged(size_t trackRate) {
    m_trackRate = static_cast<double>(trackRate);
}

void NeuvisysThread::onRangePotentialChanged(size_t rangePotential) {
    m_rangePotential = rangePotential;
}

void NeuvisysThread::onRangeSpikeTrainChanged(size_t rangeSpiketrain) {
    m_rangeSpiketrain = rangeSpiketrain;
}

void NeuvisysThread::onLayerChanged(size_t layer) {
    m_layer = layer;
    m_change = true;
}

void NeuvisysThread::onStopNetwork() {
    m_stop = true;
}