#include "neuvisysthread.h"

#include <utility>

NeuvisysThread::NeuvisysThread(int argc, char** argv, QObject *parent) : QThread(parent), m_initArgc(argc), m_initArgv(argv) {
    m_frameTime = std::chrono::high_resolution_clock::now();
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

    spinet.addLayer("SimpleCell", spinet.getNetworkConfig().SharingType, true, spinet.getNetworkConfig().layerPatches[0], spinet.getNetworkConfig().layerSizes[0], spinet.getNetworkConfig().neuronSizes[0], 0);
    spinet.addLayer("ComplexCell", "none", true, spinet.getNetworkConfig().layerPatches[1], spinet.getNetworkConfig().layerSizes[1], spinet.getNetworkConfig().neuronSizes[1], 0);
    spinet.addLayer("CriticCell", "none", false, spinet.getNetworkConfig().layerPatches[2], spinet.getNetworkConfig().layerSizes[2], spinet.getNetworkConfig().neuronSizes[2], 1);
    spinet.addLayer("ActorCell", "none", false, spinet.getNetworkConfig().layerPatches[3], spinet.getNetworkConfig().layerSizes[3], spinet.getNetworkConfig().neuronSizes[3], 1);

    spinet.loadNetwork();

    emit networkConfiguration(spinet.getNetworkConfig().SharingType, spinet.getNetworkConfig().layerPatches[0], spinet.getNetworkConfig().layerSizes[0], spinet.getNetworkConfig().neuronSizes[0]);
    emit networkCreation(spinet.getNetworkConfig().NbCameras, spinet.getNetworkConfig().Neuron1Synapses, spinet.getNetworkStructure());
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
    if (spinet.getNetworkConfig().NbCameras == 1) {
        eventPacket = mono(m_events.toStdString(), m_nbPass);
    } else if (spinet.getNetworkConfig().NbCameras == 2) {
        eventPacket = stereo(m_events.toStdString(), m_nbPass);
    }
    emit displayProgress(100, 0, 0, 0);

    for (auto event : eventPacket) {
        addEventToDisplay(event);
        spinet.runEvent(event);

        std::chrono::duration<double> frameElapsed = std::chrono::high_resolution_clock::now() - m_frameTime;
        if (1000000 * frameElapsed.count() > static_cast<double>(m_precisionEvent)) {
            m_frameTime = std::chrono::high_resolution_clock::now();
            display(spinet, eventPacket.size());
        }

        std::chrono::duration<double> trackingElapsed = std::chrono::high_resolution_clock::now() - m_trackingTime;
        if (1000000 * trackingElapsed.count() > static_cast<double>(m_precisionPotential)) {
            m_trackingTime = std::chrono::high_resolution_clock::now();
            spinet.trackNeuron(event.timestamp(), m_id, m_layer);
        }
    }

    spinet.saveNetwork(m_nbPass, m_events.toStdString());
    emit networkDestruction();
}

void NeuvisysThread::rosPass(SpikingNetwork &spinet) {
    SimulationInterface sim(1. / 150);
    std::chrono::duration<double> timeElapsed{};

    while (!m_stop) {
        ros::spinOnce();

        if (sim.hasReceivedLeftImage()) {
            sim.update();

            spinet.transmitReward(sim.getReward());
            for (const Event &event : sim.getLeftEvents()) {
                addEventToDisplay(event);
                spinet.runEvent(event);
            }

            timeElapsed = std::chrono::high_resolution_clock::now() - m_motorTime;
            if (1000000 * timeElapsed.count() > static_cast<double>(20000)) {
                m_motorTime = std::chrono::high_resolution_clock::now();

//                int selectedMotor;
//                auto exploration = sim.motorAction(spinet.resolveMotor(), 0, selectedMotor);
//                if (!sim.getLeftEvents().empty() && exploration) {
//                    auto neuron = spinet.getNeuron(selectedMotor, spinet.getNetworkStructure().size()-1);
//                    neuron.spike(sim.getLeftEvents().back().timestamp());
//                    neuron.setNeuromodulator(spinet.critic(neuron.getIndex()));
//                    neuron.weightUpdate();
//                    neuron.resetSpike();
//                }
//                if (selectedMotor != -1) {
//                    m_motorDisplay[selectedMotor] = true;
//                }
            }

            timeElapsed = std::chrono::high_resolution_clock::now() - m_frameTime;
            if (1000000 * timeElapsed.count() > static_cast<double>(m_precisionEvent)) {
                m_frameTime = std::chrono::high_resolution_clock::now();
//                display(spinet, 0);
            }

            timeElapsed = std::chrono::high_resolution_clock::now() - m_trackingTime;
            if (1000000 * timeElapsed.count() > static_cast<double>(m_precisionPotential)) {
                m_trackingTime = std::chrono::high_resolution_clock::now();
                if (!sim.getLeftEvents().empty()) {
                    spinet.trackNeuron(sim.getLeftEvents().back().timestamp(), m_id, m_layer);
                }
            }

            sim.resetLeft();
        }
    }
    spinet.saveNetwork(1, "Simulation");
    emit networkDestruction();
}

inline void NeuvisysThread::addEventToDisplay(const Event &event) {
    if (event.camera() == 0) {
        if (m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[1] == 0 && m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2] == 0) {
            m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2 - event.polarity()] = 255;
        }
    } else {
        if (m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[1] == 0 && m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2] == 0) {
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
        emit networkConfiguration(sharing, spinet.getNetworkConfig().layerPatches[m_layer], spinet.getNetworkConfig().layerSizes[m_layer], spinet.getNetworkConfig().neuronSizes[m_layer]);
    }

    if (sizeArray == 0) {
        emit displayProgress(0, spinet.getNeuron(m_id, m_layer).getSpikingRate(), spinet.getNeuron(m_id, m_layer).getThreshold(), spinet.getBias());
    } else {
        emit displayProgress(static_cast<int>(100 * m_iterations / sizeArray),
                             spinet.getNeuron(m_id, m_layer).getSpikingRate(), spinet
        .getNeuron(m_id, m_layer).getThreshold(), spinet.getBias());
    }

    for (auto xPatch : spinet.getNetworkConfig().layerPatches[0][0]) {
        for (auto yPatch : spinet.getNetworkConfig().layerPatches[0][1]) {
            auto offsetXPatch = static_cast<int>(xPatch + spinet.getNetworkConfig().layerSizes[0][0] * spinet.getNetworkConfig().neuronSizes[0][0]);
            auto offsetYPatch = static_cast<int>(yPatch + spinet.getNetworkConfig().layerSizes[0][1] * spinet.getNetworkConfig().neuronSizes[0][1]);
            cv::rectangle(m_leftEventDisplay, cv::Point(static_cast<int>(xPatch), static_cast<int>(yPatch)), cv::Point(offsetXPatch, offsetYPatch),
                          cv::Scalar(255, 0, 0));
            cv::rectangle(m_rightEventDisplay, cv::Point(static_cast<int>(xPatch), static_cast<int>(yPatch)), cv::Point(offsetXPatch, offsetYPatch),
                          cv::Scalar(255, 0, 0));
        }
    }

    emit displayEvents(m_leftEventDisplay, m_rightEventDisplay);

    prepareWeights(spinet);
    emit displayWeights(m_weightDisplay, m_layer);
    emit displaySpike(m_spikeTrain);
    emit displayPotential(spinet.getSimpleNeuronConfig().VRESET, spinet.getNeuron(m_id, m_layer).getThreshold(), spinet.getNeuron(m_id, m_layer)
    .getTrackingPotentialTrain());
    emit displayReward(spinet.getRewards());
    emit displayAction(m_motorDisplay);
    m_motorDisplay = std::vector<bool>(2, false);
    m_leftEventDisplay = 0;
    m_rightEventDisplay = 0;
}

inline void NeuvisysThread::prepareWeights(SpikingNetwork &spinet) {
    m_weightDisplay.clear();
    size_t count = 0;
    if (m_layer == 0) {
        for (size_t i = 0; i < spinet.getNetworkConfig().layerPatches[m_layer][0].size() * spinet.getNetworkConfig().layerSizes[m_layer][0]; ++i) {
            for (size_t j = 0; j < spinet.getNetworkConfig().layerPatches[m_layer][1].size() * spinet.getNetworkConfig().layerSizes[m_layer][1]; ++j) {
                m_spikeTrain[count] = spinet.getNeuron(spinet.getLayout(0, i, j, m_zcell), 0).getTrackingSpikeTrain();
                if (spinet.getNetworkConfig().SharingType == "none") {
                    m_weightDisplay[count] = spinet.getWeightNeuron(spinet.getLayout(0, i, j, m_zcell), m_layer, m_camera, m_synapse, m_zcell);
                }
                ++count;
            }
        }
        if (spinet.getNetworkConfig().SharingType == "patch") {
            count = 0;
            for (size_t wp = 0; wp < spinet.getNetworkConfig().layerPatches[m_layer][0].size(); ++wp) {
                for (size_t hp = 0; hp < spinet.getNetworkConfig().layerPatches[m_layer][1].size(); ++hp) {
                    for (size_t i = 0; i < spinet.getNetworkConfig().layerSizes[m_layer][2]; ++i) {
                        m_weightDisplay[count] = spinet.getWeightNeuron(
                                spinet.getLayout(0, wp * spinet.getNetworkConfig().layerSizes[m_layer][0], hp * spinet.getNetworkConfig().layerSizes[m_layer][1], i), m_layer, m_camera,
                                m_synapse, m_zcell);
                        ++count;
                    }
                }
            }
        } else if (spinet.getNetworkConfig().SharingType == "full") {
            for (size_t i = 0; i < spinet.getNetworkConfig().layerSizes[m_layer][2]; ++i) {
                m_weightDisplay[i] = spinet.getWeightNeuron(spinet.getLayout(0, 0, 0, i), m_layer, m_camera, m_synapse, m_zcell);
            }
        }
    } else {
        for (size_t i = 0; i < spinet.getNetworkConfig().layerSizes[m_layer][0]; ++i) {
            m_spikeTrain[count] = spinet.getNeuron(spinet.getLayout(m_layer, i, 0, m_zcell), m_layer).getTrackingSpikeTrain();
            m_weightDisplay[count] = spinet.getWeightNeuron(spinet.getLayout(m_layer, i, 0, m_zcell), m_layer, m_camera, m_synapse, m_depth);
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

void NeuvisysThread::onPrecisionEventChanged(size_t precisionEvent) {
    m_precisionEvent = precisionEvent;
}

void NeuvisysThread::onRangePotentialChanged(size_t rangePotential) {
    m_rangePotential = rangePotential;
}

void NeuvisysThread::onPrecisionPotentialChanged(size_t precisionPotential) {
    m_precisionPotential = precisionPotential;
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
