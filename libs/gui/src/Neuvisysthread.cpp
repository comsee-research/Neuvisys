//
// Created by Thomas on 14/04/2021.
//

#include "Neuvisysthread.h"

using namespace std::chrono_literals; // ns, us, ms, s, h, etc.

NeuvisysThread::NeuvisysThread(int argc, char **argv, QObject *parent) : QThread(parent), m_initArgc(argc),
                                                                         m_initArgv(argv) {
    m_iterations = 0;
    m_nbPass = 0;
}

void NeuvisysThread::render(QString networkPath, QString events, size_t nbPass, size_t mode) {
    m_networkPath = std::move(networkPath);
    m_events = std::move(events);
    m_nbPass = nbPass;
    m_iterations = 0;
    m_mode = mode;
    start(HighPriority);
}

void NeuvisysThread::run() {
    if (m_mode == 3) {
        auto network = NetworkHandle();
        m_leftEventDisplay = cv::Mat::zeros(260, 346, CV_8UC3);
        m_rightEventDisplay = cv::Mat::zeros(260, 346, CV_8UC3);
        if (m_events.toStdString().empty()) {
            readEventsRealTime();
        } else {
            network.setEventPath(m_events.toStdString());
            readEventsFile(network);
        }
    } else {
        auto network = NetworkHandle(m_networkPath.toStdString());
        m_leftEventDisplay = cv::Mat::zeros(network.getNetworkConfig().getVfHeight(), network.getNetworkConfig().getVfWidth(), CV_8UC3);
        m_rightEventDisplay = cv::Mat::zeros(network.getNetworkConfig().getVfHeight(), network.getNetworkConfig().getVfWidth(), CV_8UC3);

        emit networkConfiguration(network.getNetworkConfig().getSharingType(),
                                  network.getNetworkConfig().getLayerConnectivity()[0].patches,
                                  network.getNetworkConfig().getLayerConnectivity()[0].sizes,
                                  network.getNetworkConfig().getLayerConnectivity()[0].neuronSizes[0]);
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
            std::cout << "Mode not available, use gui-simulation target" << std::endl;
        }
    }
    emit networkDestruction();
    quit();
}

void NeuvisysThread::readEventsFile(NetworkHandle &network) {
    auto rtime = std::chrono::high_resolution_clock::now();
    auto rdisplayTime = rtime;
    auto events = std::vector<Event>();
    while (network.loadEvents(events, 1)) {
        for (const auto &event: events) {
            addEventToDisplay(event);

            if (static_cast<double>(event.timestamp()) - m_displayTime > m_displayRate) {
                m_displayTime = static_cast<double>(event.timestamp());

                m_leftEventDisplay = 0;
                m_rightEventDisplay = 0;
            }

            rtime = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration<double>(rtime - rdisplayTime).count() > m_displayRate / E6) {
                rdisplayTime = rtime;
                emit displayEvents(m_leftEventDisplay, m_rightEventDisplay);
            }
        }
    }
}

void NeuvisysThread::readEventsRealTime() {
    auto camera = EventCamera();
    auto eventFilter = Ynoise(346, 260);

    bool received = false, stop = false;
    size_t time;
    auto displayTime = 0;
    auto rtime = std::chrono::high_resolution_clock::now();
    auto rdisplayTime = rtime;

    while (!stop) {
        auto polarity = camera.receiveEvents(received, stop);
        auto events = eventFilter.run(*polarity);

        for (const auto &event: events) {
            addEventToDisplay(event);
        }

        time = events.back().timestamp();
        if (time - displayTime > static_cast<size_t>(m_displayRate)) {
            displayTime = time;

            emit displayEvents(m_leftEventDisplay, m_rightEventDisplay);
            m_leftEventDisplay = 0;
            m_rightEventDisplay = 0;
        }

//        rtime = std::chrono::high_resolution_clock::now();
//        if (std::chrono::duration<double>(rtime - rdisplayTime).count() > m_displayRate / E6) {
//            rdisplayTime = rtime;

//        }
    }
}

void NeuvisysThread::launchNetwork(NetworkHandle &network) {
    // std::vector<Event> events;

    // while (network.loadEvents(events, m_nbPass) && !m_stop) {
    //     eventLoop(network, events, static_cast<double>(events.back().timestamp()));
    // }

    // network.save(m_events.toStdString(), m_nbPass);

   std::string path_Events = "/home/comsee/Internship_Antony/neuvisys/Events/rotated_new_bars9/events/";
   std::vector<std::string> vectorOfPaths;
   for (const auto & frame : std::filesystem::directory_iterator{path_Events})
   {
       vectorOfPaths.emplace_back(frame.path().string());
   }

   network.setEventPath(vectorOfPaths[0]);
   int epochs=30;
   int numberOfTimes = 1;
   std::string typeOfTraining = "all";
   if(typeOfTraining==network.getSimpleNeuronConfig().STDP_LEARNING) {
       std::cout << "Training is about to start..." << std::endl;
       std::vector<Event> events;
       auto rng = std::default_random_engine{};
       for (int j = 0; j < epochs; j++) {
           std::shuffle(std::begin(vectorOfPaths), std::end(vectorOfPaths), rng);
           std::cout << "It's epoch number : " << j << " !" << std::endl;
           for (int i = 0; i < vectorOfPaths.size(); i++) {
               std::cout << "Training of event folder number : " << i + 1 << " !" << std::endl;
               double time = i + 1;
               while (network.loadEvents(events, numberOfTimes)) {
                   eventLoop(network, events, time);
                   break;
               }
               network.save(vectorOfPaths[i], numberOfTimes);
               events.clear();
               if (i != vectorOfPaths.size() - 1) {
                   network.setEventPath(vectorOfPaths[i + 1]);
               }
           }
           network.setEventPath(vectorOfPaths[0]);
       }
   } else{

       std::cout << "Please, verify that the type of learning is correct." << std::endl;
   }

    emit networkDestruction();
}

int NeuvisysThread::launchReal(NetworkHandle &network) {
    auto time = std::chrono::high_resolution_clock::now();

    auto camera = EventCamera();
    auto eventFilter = Ynoise(network.getNetworkConfig().getVfWidth(), network.getNetworkConfig().getVfHeight());

    bool received = false;

    while (!m_stop) {
        auto polarity = camera.receiveEvents(received, m_stop);
        auto events = eventFilter.run(*polarity);
        time = std::chrono::high_resolution_clock::now();
        if (!events.empty()) {
            eventLoop(network, events, static_cast<double>(events.back().timestamp()));
        }
    }

    // Close automatically done by destructor.
    printf("Shutdown successful.\n");
    network.save("RealTime", 1);
    return 0;
}

void NeuvisysThread::eventLoop(NetworkHandle &network, const std::vector<Event> &events, double time) {
    // if (!events.empty()) {
    //     for (auto const &event: events) {
    //         ++m_iterations;
    //         addEventToDisplay(event);
    //         network.transmitEvent(event);
    //     }
    //     network.updateNeurons(static_cast<size_t>(time));

    //     if (network.getRLConfig().getRLTraining()) {
    //         m_action = network.learningLoop(events.back().timestamp(), time, events.size(), m_msg);
    //     }
    // }

    // /*** GUI Display ***/
    // emit consoleMessage(m_msg);
    // m_msg.clear();

    // if (time - m_displayTime > m_displayRate) {
    //     m_displayTime = time;
    //     display(network, m_displayTime);
    // }

    // if (time - m_trackTime > m_trackRate) {
    //     m_trackTime = time;
    //     network.trackNeuron(time, m_id, m_layer);
    // }

    m_eventRate += static_cast<double>(events.size());
    m_displayTime = 0;
    if (!events.empty()) {
        for (auto const &event : events) {
            ++m_iterations;
        //    std::cout << "m_iteration = " << m_iterations << std::endl;
        //    addEventToDisplay(event);

                /*** GUI Display ***/
         time = event.timestamp();
        // network.updateNeurons(static_cast<size_t>(time));
        // if (  (time - m_displayTime > 0) || (m_displayTime==0)) {
        if (time - m_displayTime > m_displayRate) { 
            m_displayTime +=m_displayRate;
        /*    std::cout << "time = " << time << std::endl;
            std::cout << "m_displaytime = " << m_displayTime << std::endl;*/
            display(network, m_displayTime);
        }
        if (time - m_trackTime > m_trackRate) {
            m_trackTime = time;
            network.trackNeuron(time, m_id, m_layer);
        }

            network.transmitEvent(event);
        }

//        m_action = network.learningLoop(events.back().timestamp(), time, events.size(), m_msg);
    }

/*    if(m_displayTime==144) {
        m_displayTime = 0;
    }*/

//    emit consoleMessage(m_msg);
//    m_msg.clear();
    
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

inline void NeuvisysThread::display(NetworkHandle &network, double time) {
    if (m_change) {
        m_change = false;
        auto sharing = "none";
        if (m_layer == 0) {
            sharing = "patch";
        }
        emit networkConfiguration(sharing, network.getNetworkConfig().getLayerConnectivity()[m_layer].patches,
                                  network.getNetworkConfig().getLayerConnectivity()[m_layer].sizes,
                                  network.getNetworkConfig().getLayerConnectivity()[m_layer].neuronSizes[0]);
    }

    auto on_off_ratio = static_cast<double>(m_on_count) / static_cast<double>(m_on_count + m_off_count);
    if (m_endTime != 0) {
        emit displayProgress(static_cast<int>(100 * time / m_endTime), time);
    }
    switch (m_currentTab) {
        case 0: // event viz
            sensingZone(network);
            emit displayEvents(m_leftEventDisplay, m_rightEventDisplay);
            break;
        case 1: // statistics
            emit displayStatistics(network.getSaveData()["eventRate"], network.getSaveData()["networkRate"]);
            break;
        case 2: // weights
            prepareWeights(network);
            emit displayWeights(m_weightDisplay, m_layer);
            break;
        case 3: // potential
            emit displayPotential(network.getNeuron(m_id, m_layer).get().getSpikingRate(),
                                  network.getSimpleNeuronConfig().VRESET,
                                  network.getNeuron(m_id, m_layer).get().getThreshold(),
                                  network.getNeuron(m_id, m_layer).get().getTrackingPotentialTrain());
            break;
        case 4: // spiketrain
            prepareSpikes(network);
            emit displaySpike(m_spikeTrain, time);
            break;
        case 5: // reward
            emit displayReward(network.getSaveData()["reward"], network.getSaveData()["value"], network.getSaveData()["valueDot"],
                               network.getSaveData()["tdError"]);
            break;
        case 6: // action
            emit displayAction(network.getSaveData()["action_0"], network.getSaveData()["action_1"]);
        default:
            break;
    }
    m_on_count = 0;
    m_off_count = 0;
    m_motorDisplay = std::vector<bool>(network.getNetworkStructure().back(), false);
    m_leftEventDisplay = 0;
    m_rightEventDisplay = 0;
}

inline void NeuvisysThread::sensingZone(NetworkHandle &network) {
    for (size_t i = 0; i < network.getNetworkConfig().getLayerConnectivity()[0].patches[0].size(); ++i) {
        for (size_t j = 0; j < network.getNetworkConfig().getLayerConnectivity()[0].patches[1].size(); ++j) {
            auto offsetXPatch = static_cast<int>(network.getNetworkConfig().getLayerConnectivity()[0].patches[0][i] +
                                                 network.getNetworkConfig().getLayerConnectivity()[0].sizes[0] *
                                                 network.getNetworkConfig().getLayerConnectivity()[0].neuronSizes[0][0]);
            auto offsetYPatch = static_cast<int>(network.getNetworkConfig().getLayerConnectivity()[0].patches[1][j] +
                                                 network.getNetworkConfig().getLayerConnectivity()[0].sizes[1] *
                                                 network.getNetworkConfig().getLayerConnectivity()[0].neuronSizes[0][1]);
            cv::rectangle(m_leftEventDisplay, cv::Point(static_cast<int>(network.getNetworkConfig().getLayerConnectivity()[0].patches[0][i]),
                                                        static_cast<int>(network.getNetworkConfig().getLayerConnectivity()[0].patches[1][j])),
                          cv::Point(offsetXPatch, offsetYPatch), cv::Scalar(255, 0, 0));
            cv::rectangle(m_rightEventDisplay, cv::Point(static_cast<int>(network.getNetworkConfig().getLayerConnectivity()[0].patches[0][i]),
                                                         static_cast<int>(network.getNetworkConfig().getLayerConnectivity()[0].patches[1][j])),
                          cv::Point(offsetXPatch, offsetYPatch), cv::Scalar(255, 0, 0));
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
        for (size_t i = 0; i < network.getNetworkConfig().getLayerConnectivity()[m_layer].patches[0].size() *
                               network.getNetworkConfig().getLayerConnectivity()[m_layer].sizes[0]; ++i) {
            for (size_t j = 0; j < network.getNetworkConfig().getLayerConnectivity()[m_layer].patches[1].size() *
                                   network.getNetworkConfig().getLayerConnectivity()[m_layer].sizes[1]; ++j) {
                if (network.getNetworkConfig().getSharingType() == "none") {
                    m_weightDisplay[count] = network.neuronWeightMatrix(network.getLayout(0, Position(i, j, m_zcell)),
                                                                        m_layer, m_camera, m_synapse, m_zcell);
                }
                ++count;
            }
        }
        if (network.getNetworkConfig().getSharingType() == "patch") {
            count = 0;
            for (size_t wp = 0; wp < network.getNetworkConfig().getLayerConnectivity()[m_layer].patches[0].size(); ++wp) {
                for (size_t hp = 0; hp < network.getNetworkConfig().getLayerConnectivity()[m_layer].patches[1].size(); ++hp) {
                    for (size_t i = 0; i < network.getNetworkConfig().getLayerConnectivity()[m_layer].sizes[2]; ++i) {
                        m_weightDisplay[count] = network.neuronWeightMatrix(
                                network.getLayout(0, Position(wp * network.getNetworkConfig().getLayerConnectivity()[m_layer].sizes[0],
                                                              hp * network.getNetworkConfig().getLayerConnectivity()[m_layer].sizes[1],
                                                              i)), m_layer, m_camera, m_synapse, m_zcell);
                        ++count;
                    }
                }
            }
        } else if (network.getNetworkConfig().getSharingType() == "full") {
            count = 0;
            for (size_t i = 0; i < network.getNetworkConfig().getLayerConnectivity()[m_layer].sizes[2]; ++i) {
                m_weightDisplay[count] = network.neuronWeightMatrix(
                        network.getLayout(0, Position(0, 0, i)), m_layer, m_camera, m_synapse, m_zcell);
                ++count;
            }
        }
    } else {
    //    std::cout << "yooo!" << std::endl;
    //    std::cout << "layer number = " << m_layer << std::endl;
        for (size_t i = 0; i < network.getNetworkConfig().getLayerConnectivity()[m_layer].sizes[0]; ++i) {
            m_weightDisplay[count] = network.getSummedWeightNeuron(network.getLayout(m_layer, Position(i, 0, m_zcell)), m_layer);
            ++count;
        //    std::cout << "count = " << count << std::endl;
        }
    }
}

//int NeuvisysThread::launchReal(NetworkHandle &network) {
//    auto time = std::chrono::high_resolution_clock::now();
//    auto motorTime = time;
//    auto positionTime = time;
//
//    BrushlessMotor lXMotor(0, "/dev/ttyUSB0");
//    lXMotor.setBounds(-55000, 55000);
//
//    std::vector<double> motorMapping;
//    motorMapping.emplace_back(350); // left horizontal -> left movement
//    motorMapping.emplace_back(0); // no movement
//    motorMapping.emplace_back(-350); // left horizontal  -> right movement
//
//    auto camera = EventCamera();
//    auto eventFilter = Ynoise(network.getNetworkConfig().getVfWidth(), network.getNetworkConfig().getVfHeight());
//
//    double position = 0;
//    double reward = 0;
//    bool received = false;
//
//    while (!m_stop) {
//        auto polarity = camera.receiveEvents(received, m_stop);
//
//        auto dt = std::chrono::duration_cast<std::chrono::seconds>(time - std::chrono::high_resolution_clock::now()).count();
//        auto timeSec = static_cast<double>(std::chrono::time_point_cast<std::chrono::microseconds>(time).time_since_epoch().count()) / E6;
//        lXMotor.jitterSpeed(static_cast<double>(dt));
//
//        if (lXMotor.isActionValid(position, 0)) {
//            reward = 80 * (55000 - abs(position)) / 55000;
//        } else {
//            reward = -100;
//        }
//
//        time = std::chrono::high_resolution_clock::now();
//        eventLoop(network, eventFilter.run(*polarity), std::chrono::time_point_cast<std::chrono::microseconds>(time).time_since_epoch().count());
//
//        if (m_action != -1) {
//            position += motorMapping[m_action] * std::chrono::duration_cast<std::chrono::seconds>(time - motorTime).count();
//            if (lXMotor.isActionValid(position, 0)) {
//                motorTime = time;
//                lXMotor.setSpeed(motorMapping[m_action]);
//            } else {
//                lXMotor.setSpeed(0);
//            }
//        }
//
//        if (std::chrono::duration<double>(time - positionTime).count() > 3.0) {
//            positionTime = time;
//            std::cout << "Before: " << position << std::endl;
//            position = lXMotor.getPosition();
//            std::cout << "After: " << position << std::endl;
//        }
//    }
//
//    // Close automatically done by destructor.
//    printf("Shutdown successful.\n");
//    network.save("Simulation", 1);
//    return 0;
//}

void NeuvisysThread::onTabVizChanged(size_t index) {
    m_currentTab = index;
}

void NeuvisysThread::onIndexChanged(size_t index) {
    m_id = index;
}

void NeuvisysThread::onZcellChanged(size_t zcell) {
    m_zcell = zcell;
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
    m_id = 0;
    m_zcell = 0;
    m_change = true;
}

void NeuvisysThread::onStopNetwork() {
    m_stop = true;
}
