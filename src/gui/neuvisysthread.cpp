#include "neuvisysthread.h"

NeuvisysThread::NeuvisysThread(QObject *parent) : QThread(parent) {
    m_frameTime = std::chrono::high_resolution_clock::now();
    m_iterations = 0;
    m_nbPass = 0;
}

void NeuvisysThread::run() {
    std::cout << "Initializing Network " << std::endl;
    SpikingNetwork spinet(m_networkPath.toStdString());

    emit networkConfiguration(spinet.getNetworkConfig().NbCameras, spinet.getNetworkConfig().Neuron1Synapses, spinet.getNetworkConfig().SharingType, spinet.getNetworkConfig().L1XAnchor.size() * spinet.getNetworkConfig().L1Width, spinet.getNetworkConfig().L1YAnchor.size() * spinet.getNetworkConfig().L1Height, spinet.getNetworkConfig().L1Depth, spinet.getNetworkConfig().L1XAnchor.size(), spinet.getNetworkConfig().L1YAnchor.size());
    m_leftEventDisplay = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);
    m_rightEventDisplay = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);

    if (1) {
        multiplePass(spinet);
    } else if (2) {
        rosPass(spinet);
    }

    quit();
}

void NeuvisysThread::render(QString networkPath, QString events, size_t nbPass) {
    m_networkPath = std::move(networkPath);
    m_events = std::move(events);
    m_nbPass = nbPass;
    m_iterations = 0;
    start(HighPriority);
}

void NeuvisysThread::multiplePass(SpikingNetwork &spinet) {
    auto eventPacket = std::vector<Event>();
    if (spinet.getNetworkConfig().NbCameras == 1) {
        eventPacket = mono(m_events.toStdString(), m_nbPass);
    } else if (spinet.getNetworkConfig().NbCameras == 2) {
        eventPacket = stereo(m_events.toStdString(), m_nbPass);
    }
    emit displayInformation(100, 0, 0, 0, m_leftEventDisplay, m_rightEventDisplay, m_weightDisplay, std::vector<std::pair<double, long>>(), m_spikeTrain);
    runSpikingNetwork(spinet, eventPacket, 0);
    std::cout << "Finished" << std::endl;
}

void NeuvisysThread::rosPass(SpikingNetwork &spinet) {
    SimulationInterface sim(spinet);

    auto start = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::now();
    while(std::chrono::duration_cast<std::chrono::milliseconds>(time - start).count() < 60000) {
        time = std::chrono::system_clock::now();
        ros::spinOnce();

        if (sim.hasReceivedLeftImage()) {
            auto events = sim.getLeftEvents();
            runSpikingNetwork(spinet, events, sim.getReward());
            sim.activateMotors(spinet.getMotorActivation(), 0);

            spinet.resetMotorActivation();
            sim.resetLeft();
        }
        if (sim.hasReceivedRightImage()) {
            auto events = sim.getRightEvents();
            runSpikingNetwork(spinet, events, sim.getReward());
            sim.activateMotors(spinet.getMotorActivation(), 0);

            spinet.resetMotorActivation();
            sim.resetRight();
        }
    }
}

inline void NeuvisysThread::runSpikingNetwork(SpikingNetwork &spinet, const std::vector<Event> &eventPacket, const double reward) {
    spinet.setReward(reward);
    spinet.resetMotorActivation();
    for (Event event : eventPacket) {
        // Display event-based image
        if (event.camera() == 0) {
            if (m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[1] == 0 && m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2] == 0) {
                m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2-event.polarity()] = 255;
            }
        } else {
            if (m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[1] == 0 && m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2] == 0) {
                m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2-event.polarity()] = 255;
            }
        }

        spinet.addEvent(event);

//    if (count % Conf::EVENT_FREQUENCY == 0) {
//        spinet.updateNeurons(event.timestamp());
//    }

        std::chrono::duration<double> frameElapsed = std::chrono::high_resolution_clock::now() - m_frameTime;
        if (1000000 * frameElapsed.count() > static_cast<double>(m_precisionEvent)) {
            m_frameTime = std::chrono::high_resolution_clock::now();
            display(spinet, eventPacket.size());
        }

        std::chrono::duration<double> trackingElapsed = std::chrono::high_resolution_clock::now() - m_trackingTime;
        if (1000000 * trackingElapsed.count() > static_cast<double>(m_precisionPotential)) {
            m_trackingTime = std::chrono::high_resolution_clock::now();
            spinet.trackNeuron(event.timestamp(), m_idSimple, m_idComplex);
        }

        if (static_cast<size_t>(m_iterations) % Conf::UPDATE_PARAMETER_FREQUENCY == 0) {
            spinet.updateNeuronsParameters(event.timestamp());
        }
        ++m_iterations;
    }
}

inline void NeuvisysThread::display(SpikingNetwork &spinet, size_t sizeArray) {
    for (auto xPatch : spinet.getNetworkConfig().L1XAnchor) {
        for (auto yPatch : spinet.getNetworkConfig().L1YAnchor) {
            auto offsetXPatch = static_cast<int>(xPatch + spinet.getNetworkConfig().L1Width * spinet.getNetworkConfig().Neuron1Width);
            auto offsetYPatch = static_cast<int>(yPatch + spinet.getNetworkConfig().L1Height * spinet.getNetworkConfig().Neuron1Height);
            cv::rectangle(m_leftEventDisplay, cv::Point(static_cast<int>(xPatch), static_cast<int>(yPatch)), cv::Point(offsetXPatch, offsetYPatch), cv::Scalar(255, 0, 0));
            cv::rectangle(m_rightEventDisplay, cv::Point(static_cast<int>(xPatch), static_cast<int>(yPatch)), cv::Point(offsetXPatch, offsetYPatch), cv::Scalar(255, 0, 0));
        }
    }

    size_t count = 0;
    for (size_t i = 0; i < spinet.getNetworkConfig().L1XAnchor.size() * spinet.getNetworkConfig().L1Width; ++i) {
        for (size_t j = 0; j < spinet.getNetworkConfig().L1YAnchor.size() * spinet.getNetworkConfig().L1Height; ++j) {
            m_spikeTrain[count] = spinet.getSpikingNeuron(spinet.getLayout1(i, j, m_layerSimple), 0);
            if (spinet.getNetworkConfig().SharingType == "none") {
                m_weightDisplay[count] = spinet.getWeightNeuron(spinet.getLayout1(i, j, m_layerSimple), m_camera, m_synapse, 0, 0);
            }
            ++count;
        }
    } if (spinet.getNetworkConfig().SharingType == "patch") {
        count = 0;
        for (size_t wp = 0; wp < spinet.getNetworkConfig().L1XAnchor.size(); ++wp) {
            for (size_t hp = 0; hp < spinet.getNetworkConfig().L1YAnchor.size(); ++hp) {
                for (size_t i = 0; i < spinet.getNetworkConfig().L1Depth; ++i) {
                    m_weightDisplay[count] = spinet.getWeightNeuron(spinet.getLayout1(wp*spinet.getNetworkConfig().L1Width, hp*spinet.getNetworkConfig().L1Height, i), m_camera, m_synapse, 0, 0);
                    ++count;
                }
            }
        }
    } else if (spinet.getNetworkConfig().SharingType == "full") {
        for (size_t i = 0; i < spinet.getNetworkConfig().L1Depth; ++i) {
            m_weightDisplay[i] = spinet.getWeightNeuron(spinet.getLayout1(0, 0, i), m_camera, m_synapse, 0, 0);
        }
    }
    emit displayInformation(static_cast<int>(100 * m_iterations / sizeArray), 1000 * spinet.getNeuron(m_idSimple).getSpikingRate(), spinet.getNeuron(m_idSimple).getThreshold(), spinet.getSimpleNeuronConfig().VRESET, m_leftEventDisplay, m_rightEventDisplay, m_weightDisplay, spinet.getPotentialNeuron(m_idSimple, 0), m_spikeTrain);
    m_leftEventDisplay = 0;
    m_rightEventDisplay = 0;
}

void NeuvisysThread::onGuiInformation(const size_t index, const size_t layer, const size_t camera, const size_t synapse, const size_t precisionEvent, const size_t rangePotential, const size_t precisionPotential, const size_t rangeSpiketrain) {
    m_idSimple = index;
    m_layerSimple = layer;
    m_camera = camera;
    m_synapse = synapse;

    m_precisionEvent = precisionEvent;
    m_rangePotential = rangePotential;
    m_precisionPotential = precisionPotential;
    m_rangeSpiketrain = rangeSpiketrain;
}
