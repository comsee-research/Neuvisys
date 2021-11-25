#include "neuvisysthread.h"
#include "../robot-control/motor-control/StepMotor.hpp"
#include "../dv-modules/DavisHandle.hpp"
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
//        launchSimulation(network);
        launchReal(network);
    } else {
        launchNetwork(network);
    }
    quit();
}

void NeuvisysThread::launchNetwork(NetworkHandle &network) {
    auto eventPacket = std::vector<Event>();
    if (network.getNetworkConfig().getNbCameras() == 1) {
        eventPacket = NetworkHandle::mono(m_events.toStdString(), m_nbPass);
    } else if (network.getNetworkConfig().getNbCameras() == 2) {
        eventPacket = NetworkHandle::stereo(m_events.toStdString(), m_nbPass);
    }
    emit displayProgress(100, 0, 0, 0, 0, 0, 0);

    for (const auto &event: eventPacket) {
        addEventToDisplay(event);
        network.transmitEvent(event);

//        if () { event clock (30ms)
//            display(network, eventPacket.size());
//        }
//
//        if () { event clock (10ms)
//            network.trackNeuron(event.timestamp(), m_id, m_layer);
//        }
    }

    network.save(m_nbPass, m_events.toStdString());
    emit networkDestruction();
}

void NeuvisysThread::launchSimulation(NetworkHandle &network) {
    SimulationInterface sim;
    sim.enableSyncMode(true);
    sim.startSimulation();

    m_simTimeStep = static_cast<size_t>(sim.getSimulationTimeStep());

    int action = 0;
    double displayTime = 0, trackTime = 0;
    std::string msg;

    while (!m_stop) {
        sim.triggerNextTimeStep();
        while (!sim.simStepDone() && !m_stop) {
            ros::spinOnce();
        }

        sim.update();
        if (!sim.getLeftEvents().empty()) {
            m_eventRate += static_cast<double>(sim.getLeftEvents().size());
            action = network.mainLoop(sim.getLeftEvents(), sim.getReward(), sim.getSimulationTime(), msg);

            if (action != -1) {
                sim.activateMotors(action);
                m_motorDisplay[action] = true;
            }
            emit consoleMessage(msg);

            /*** GUI Display ***/
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

int NeuvisysThread::launchReal(NetworkHandle &network) {
    std::chrono::high_resolution_clock::time_point time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point updateTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point actionTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point consoleTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point motorTime = std::chrono::high_resolution_clock::now();

    std::chrono::high_resolution_clock::time_point displayTime = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point trackTime = std::chrono::high_resolution_clock::now();

    StepMotor motor = StepMotor("leftmotor1", 0, "/dev/ttyUSB0");
    motor.setBounds(-55000, 55000);

    std::vector<double> motorMapping;
    motorMapping.emplace_back(350); // left horizontal -> left movement
    motorMapping.emplace_back(0); // no movement
    motorMapping.emplace_back(-350); // left horizontal  -> right movement

    // Install signal handler for global shutdown.
    struct sigaction shutdownAction;

    shutdownAction.sa_handler = &globalShutdownSignalHandler;
    shutdownAction.sa_flags = 0;
    sigemptyset(&shutdownAction.sa_mask);
    sigaddset(&shutdownAction.sa_mask, SIGTERM);
    sigaddset(&shutdownAction.sa_mask, SIGINT);

    if (sigaction(SIGTERM, &shutdownAction, NULL) == -1) {
        libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
                          "Failed to set signal handler for SIGTERM. Error: %d.", errno);
        return (EXIT_FAILURE);
    }

    if (sigaction(SIGINT, &shutdownAction, NULL) == -1) {
        libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
                          "Failed to set signal handler for SIGINT. Error: %d.", errno);
        return (EXIT_FAILURE);
    }

    // Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
    libcaer::devices::davis davis = libcaer::devices::davis(1);

    // Let's take a look at the information we have on the device.
    struct caer_davis_info davis_info = davis.infoGet();

    printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", davis_info.deviceString,
           davis_info.deviceID, davis_info.deviceIsMaster, davis_info.dvsSizeX, davis_info.dvsSizeY,
           davis_info.logicVersion);

    // Send the default configuration before using the device.
    // No configuration is sent automatically!
    davis.sendDefaultConfig();

    // Tweak some biases, to increase bandwidth in this case.
    struct caer_bias_coarsefine coarseFineBias;

    coarseFineBias.coarseValue = 2;
    coarseFineBias.fineValue = 116;
    coarseFineBias.enabled = true;
    coarseFineBias.sexN = false;
    coarseFineBias.typeNormal = true;
    coarseFineBias.currentLevelNormal = true;

    davis.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias));

    coarseFineBias.coarseValue = 1;
    coarseFineBias.fineValue = 33;
    coarseFineBias.enabled = true;
    coarseFineBias.sexN = false;
    coarseFineBias.typeNormal = true;
    coarseFineBias.currentLevelNormal = true;

    davis.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias));

    // Let's verify they really changed!
    uint32_t prBias = davis.configGet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP);
    uint32_t prsfBias = davis.configGet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP);

    printf("New bias values --- PR-coarse: %d, PR-fine: %d, PRSF-coarse: %d, PRSF-fine: %d.\n",
           caerBiasCoarseFineParse(prBias).coarseValue, caerBiasCoarseFineParse(prBias).fineValue,
           caerBiasCoarseFineParse(prsfBias).coarseValue, caerBiasCoarseFineParse(prsfBias).fineValue);

    // Now let's get start getting some data from the device. We just loop in blocking mode,
    // no notification needed regarding new events. The shutdown notification, for example if
    // the device is disconnected, should be listened to.
    davis.dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);

    // Let's turn on blocking data-get mode to avoid wasting resources.
    davis.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

    std::vector<size_t> vecEvents;

    double position = 0;
    int action = 0;
    double reward = 0;
    std::string msg;
    while (!globalShutdown.load(memory_order_relaxed)) {
        std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = davis.dataGet();
        if (packetContainer == nullptr) {
            continue; // Skip if nothing there.
        }

        for (auto &packet : *packetContainer) {
            if (packet == nullptr) {
                continue; // Skip if nothing there.
            }

            if (packet->getEventType() == POLARITY_EVENT) {
                time = std::chrono::high_resolution_clock::now();
                std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
                        = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

                if (motor.isActionValid(position, 0)) {
                    reward = 80 * (55000 - abs(position)) / 55000;
                } else {
                    reward = -100;
                }

                network.mainLoop(*polarity, reward, time, msg);

//                for (const auto &eve : *polarity) {
//
//                }

                if (action != -1) {
                    if (motor.isActionValid(position, 0)) {
                        motor.setSpeed(motorMapping[action]);;
                    } else {
                        motor.setSpeed(0);
                    }
                }

                /*** GUI Display ***/
                if (std::chrono::duration<double>(time - displayTime).count() > m_displayRate / E6) {
                    displayTime = std::chrono::high_resolution_clock::now();
                    display(network, 0, 0);
                }

                if (std::chrono::duration<double>(time - trackTime).count() > m_displayRate / E6) {
                    trackTime = std::chrono::high_resolution_clock::now();
                    if (!polarity->empty()) {
                        network.trackNeuron(polarity->back().getTimestamp(), m_id, m_layer);
                    }
                }

                if (std::chrono::duration<double>(time - motorTime).count() > 1) {
                    motorTime = std::chrono::high_resolution_clock::now();
                    position = motor.getPosition();
                }
            }
        }
    }
    davis.dataStop();

    // Close automatically done by destructor.
    printf("Shutdown successful.\n");
    network.save(1, "Simulation");
    return 0;
}