#include "neuvisysthread.h"

#include <utility>

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
    auto spinet = SpikingNetwork(m_networkPath.toStdString());

    spinet.addLayer("SimpleCell", spinet.getNetworkConfig().getSharingType(), true,
                    spinet.getNetworkConfig().getLayerPatches()[0], spinet.getNetworkConfig().getLayerSizes()[0],
                    spinet.getNetworkConfig().getNeuronSizes()[0], spinet.getNetworkConfig().getNeuronOverlap()[0], 0);
    spinet.addLayer("ComplexCell", "none", true, spinet.getNetworkConfig().getLayerPatches()[1],
                    spinet.getNetworkConfig().getLayerSizes()[1], spinet.getNetworkConfig().getNeuronSizes()[1],
                    spinet.getNetworkConfig().getNeuronOverlap()[1], 0);
    spinet.addLayer("CriticCell", "none", false, spinet.getNetworkConfig().getLayerPatches()[2],
                    spinet.getNetworkConfig().getLayerSizes()[2], spinet.getNetworkConfig().getNeuronSizes()[2],
                    spinet.getNetworkConfig().getNeuronOverlap()[2], 1);
    spinet.addLayer("ActorCell", "none", true, spinet.getNetworkConfig().getLayerPatches()[3],
                    spinet.getNetworkConfig().getLayerSizes()[3], spinet.getNetworkConfig().getNeuronSizes()[3],
                    spinet.getNetworkConfig().getNeuronOverlap()[3], 1);

    spinet.loadNetwork();

    emit networkConfiguration(spinet.getNetworkConfig().getSharingType(),
                              spinet.getNetworkConfig().getLayerPatches()[0],
                              spinet.getNetworkConfig().getLayerSizes()[0],
                              spinet.getNetworkConfig().getNeuronSizes()[0]);
    emit networkCreation(spinet.getNetworkConfig().getNbCameras(), spinet.getNetworkConfig().getNeuron1Synapses(),
                         spinet.getNetworkStructure());
    m_leftEventDisplay = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);
    m_rightEventDisplay = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);
    m_motorDisplay = std::vector<bool>(2, false);

    if (m_realtime) {
        rosPass(spinet);
    } else {
        multiplePass(spinet);
    }
    quit();
}

void NeuvisysThread::multiplePass(SpikingNetwork &spinet) {
    auto eventPacket = std::vector<Event>();
    if (spinet.getNetworkConfig().getNbCameras() == 1) {
        eventPacket = mono(m_events.toStdString(), m_nbPass);
    } else if (spinet.getNetworkConfig().getNbCameras() == 2) {
        eventPacket = stereo(m_events.toStdString(), m_nbPass);
    }
    emit displayProgress(100, 0, 0, 0, 0, 0, 0);

    for (auto event: eventPacket) {
        addEventToDisplay(event);
        spinet.runEvent(event);

//        if () { event clock (30ms)
//            display(spinet, eventPacket.size());
//        }

//        if () { event clock (10ms)
//            spinet.trackNeuron(event.timestamp(), m_id, m_layer);
//        }
    }

    spinet.saveNetwork(m_nbPass, m_events.toStdString());
    emit networkDestruction();
}

void NeuvisysThread::rosPass(SpikingNetwork &spinet) {
    SimulationInterface sim(1. / 150);
    sim.enableSyncMode(true);
    sim.startSimulation();

    double actionTime = 0, displayTime = 0, trackTime = 0;
    while (!m_stop) {
        sim.triggerNextTimeStep();
        while (!sim.simStepDone()) {
            ros::spinOnce();
        }

        m_simTime = sim.getSimulationTime();
        sim.update();
        spinet.transmitReward(sim.getReward());
        m_eventRate += static_cast<double>(sim.getLeftEvents().size());
        for (const Event &event: sim.getLeftEvents()) {
            addEventToDisplay(event);
            spinet.runEvent(event);
        }
        if (!sim.getLeftEvents().empty()) {
            spinet.updateTDError(sim.getLeftEvents().back().timestamp());
        }

        if (sim.getSimulationTime() - actionTime > m_actionRate / Conf::E6) {
            actionTime = sim.getSimulationTime();
            if (m_actor != -1) {
                auto neuron = spinet.getNeuron(m_actor, spinet.getNetworkStructure().size() - 1);
                neuron.get().spike(sim.getLeftEvents().back().timestamp());
                neuron.get().setNeuromodulator(spinet.updateTDError());
                neuron.get().weightUpdate();
            }
            sim.motorAction(spinet.resolveMotor(), 0, m_actor);

            if (m_actor != -1) {
                m_motorDisplay[m_actor] = true;
            }
        }

        if (sim.getSimulationTime() - displayTime > m_displayRate / Conf::E6) {
            displayTime = sim.getSimulationTime();
            display(spinet, 0);
        }

        if (sim.getSimulationTime() - trackTime > m_trackRate / Conf::E6) {
            trackTime = sim.getSimulationTime();
            if (!sim.getLeftEvents().empty()) {
                spinet.trackNeuron(sim.getLeftEvents().back().timestamp(), m_id, m_layer);
            }
        }
    }

    sim.stopSimulation();
    spinet.saveNetwork(1, "Simulation");
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

inline void NeuvisysThread::display(SpikingNetwork &spinet, size_t sizeArray) {
    if (m_change) {
        m_change = false;
        auto sharing = "none";
        if (m_layer == 0) {
            sharing = "patch";
        }
        emit networkConfiguration(sharing, spinet.getNetworkConfig().getLayerPatches()[m_layer],
                                  spinet.getNetworkConfig().getLayerSizes()[m_layer],
                                  spinet.getNetworkConfig().getNeuronSizes()[m_layer]);
    }

    m_eventRate = (1e6 / m_displayRate) * m_eventRate;
    auto on_off_ratio = static_cast<double>(m_on_count) / static_cast<double>(m_on_count + m_off_count);
    if (sizeArray == 0) {
        emit displayProgress(0, m_simTime, m_eventRate, on_off_ratio,
                             spinet.getNeuron(m_id, m_layer).get().getSpikingRate(),
                             spinet.getNeuron(m_id, m_layer).get().getThreshold(), spinet.getBias());
    } else {
        emit displayProgress(static_cast<int>(100 * m_iterations / sizeArray), m_simTime, m_eventRate, on_off_ratio,
                             spinet.getNeuron(m_id, m_layer).get().getSpikingRate(), spinet
                                     .getNeuron(m_id, m_layer).get().getThreshold(), spinet.getBias());
    }
    m_eventRate = 0;
    m_on_count = 0;
    m_off_count = 0;

    // add rectangle to sensing zone
    for (size_t i = 0; i < spinet.getNetworkConfig().getLayerPatches()[0][0].size(); ++i) {
        for (size_t j = 0; j < spinet.getNetworkConfig().getLayerPatches()[0][1].size(); ++j) {
            auto offsetXPatch = static_cast<int>(spinet.getNetworkConfig().getLayerPatches()[0][0][i] +
                                                 spinet.getNetworkConfig().getLayerSizes()[0][0] *
                                                 spinet.getNetworkConfig().getNeuronSizes()[0][0]);
            auto offsetYPatch = static_cast<int>(spinet.getNetworkConfig().getLayerPatches()[0][1][j] +
                                                 spinet.getNetworkConfig().getLayerSizes()[0][1] *
                                                 spinet.getNetworkConfig().getNeuronSizes()[0][1]);
            cv::rectangle(m_leftEventDisplay,
                          cv::Point(static_cast<int>(spinet.getNetworkConfig().getLayerPatches()[0][0][i]),
                                    static_cast<int>(spinet.getNetworkConfig().getLayerPatches()[0][1][j])),
                          cv::Point(offsetXPatch, offsetYPatch),
                          cv::Scalar(255, 0, 0));
            cv::rectangle(m_rightEventDisplay,
                          cv::Point(static_cast<int>(spinet.getNetworkConfig().getLayerPatches()[0][0][i]),
                                    static_cast<int>(spinet.getNetworkConfig().getLayerPatches()[0][1][j])),
                          cv::Point(offsetXPatch, offsetYPatch),
                          cv::Scalar(255, 0, 0));
        }
    }
    emit displayEvents(m_leftEventDisplay, m_rightEventDisplay);

    prepareWeights(spinet);
    emit displayWeights(m_weightDisplay, m_layer);
    emit displaySpike(m_spikeTrain);
    emit displayPotential(spinet.getSimpleNeuronConfig().VRESET, spinet.getNeuron(m_id, m_layer).get().getThreshold(),
                          spinet.getNeuron(m_id, m_layer).get().getTrackingPotentialTrain());
    emit displayReward(spinet.getRewards(), spinet.getListValue(), spinet.getListValueDot(), spinet.getListTDError());
    emit displayAction(m_motorDisplay);
    m_motorDisplay = std::vector<bool>(spinet.getNetworkStructure().back(), false);
    m_leftEventDisplay = 0;
    m_rightEventDisplay = 0;
}

inline void NeuvisysThread::prepareWeights(SpikingNetwork &spinet) {
    m_weightDisplay.clear();
    size_t count = 0;
    if (m_layer == 0) {
        for (size_t i = 0; i < spinet.getNetworkConfig().getLayerPatches()[m_layer][0].size() *
                               spinet.getNetworkConfig().getLayerSizes()[m_layer][0]; ++i) {
            for (size_t j = 0; j < spinet.getNetworkConfig().getLayerPatches()[m_layer][1].size() *
                                   spinet.getNetworkConfig().getLayerSizes()[m_layer][1]; ++j) {
                m_spikeTrain[count] = spinet.getNeuron(spinet.getLayout(0, Position(i, j, m_zcell)),
                                                       0).get().getTrackingSpikeTrain();
                if (spinet.getNetworkConfig().getSharingType() == "none") {
                    m_weightDisplay[count] = spinet.getWeightNeuron(spinet.getLayout(0, Position(i, j, m_zcell)),
                                                                    m_layer, m_camera, m_synapse, m_zcell);
                }
                ++count;
            }
        }
        if (spinet.getNetworkConfig().getSharingType() == "patch") {
            count = 0;
            for (size_t wp = 0; wp < spinet.getNetworkConfig().getLayerPatches()[m_layer][0].size(); ++wp) {
                for (size_t hp = 0; hp < spinet.getNetworkConfig().getLayerPatches()[m_layer][1].size(); ++hp) {
                    for (size_t i = 0; i < spinet.getNetworkConfig().getLayerSizes()[m_layer][2]; ++i) {
                        m_weightDisplay[count] = spinet.getWeightNeuron(
                                spinet.getLayout(0, Position(wp * spinet.getNetworkConfig().getLayerSizes()[m_layer][0],
                                                             hp * spinet.getNetworkConfig().getLayerSizes()[m_layer][1],
                                                             i)), m_layer, m_camera,
                                m_synapse, m_zcell);
                        ++count;
                    }
                }
            }
        } else if (spinet.getNetworkConfig().getSharingType() == "full") {
            for (size_t i = 0; i < spinet.getNetworkConfig().getLayerSizes()[m_layer][2]; ++i) {
                m_weightDisplay[i] = spinet.getWeightNeuron(spinet.getLayout(0, Position(0, 0, i)), m_layer, m_camera,
                                                            m_synapse, m_zcell);
            }
        }
    } else {
        for (size_t i = 0; i < spinet.getNetworkConfig().getLayerSizes()[m_layer][0]; ++i) {
            m_spikeTrain[count] = spinet.getNeuron(spinet.getLayout(m_layer, Position(i, 0, m_zcell)),
                                                   m_layer).get().getTrackingSpikeTrain();
            m_weightDisplay[count] = spinet.getWeightNeuron(spinet.getLayout(m_layer, Position(i, 0, m_zcell)), m_layer,
                                                            m_camera, m_synapse, m_depth);
            ++count;
        }
    }
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
