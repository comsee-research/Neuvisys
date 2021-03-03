#include "neuvisysthread.h"

NeuvisysThread::NeuvisysThread(QObject *parent) : QThread(parent) {
    m_frameTime = std::chrono::high_resolution_clock::now();
    m_iterations = 0;
}

void NeuvisysThread::run() {
    multiplePass();
    quit();
}

void NeuvisysThread::render(QString networkPath, QString events, int nbPass) {
    m_networkPath = networkPath;
    m_events = events;
    m_nbPass = nbPass;
    m_iterations = 0;
    start(HighPriority);
}

void NeuvisysThread::multiplePass() {
    NetworkConfig config = NetworkConfig(m_networkPath.toStdString());
    std::cout << "Initializing Network " << std::endl;
    SpikingNetwork spinet(config);
    emit networkConfiguration(config.NbCameras, config.Neuron1Synapses, config.SharingType, config.L1XAnchor.size() * config.L1Width, config.L1YAnchor.size() * config.L1Height, config.L1Depth, config.L1XAnchor.size(), config.L1YAnchor.size());

    m_leftEventDisplay = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);
    m_rightEventDisplay = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);

    main_loop(spinet);

    emit displayInformation(100, 0, 0, m_leftEventDisplay, m_rightEventDisplay, m_weightDisplay, std::vector<std::pair<double, long>>(), m_spikeTrain);
    std::cout << "Finished" << std::endl;
}

void NeuvisysThread::main_loop(SpikingNetwork &spinet) {
    size_t pass, left, right;

    cnpy::NpyArray l_timestamps_array = cnpy::npz_load(m_events.toStdString(), "arr_0");
    cnpy::NpyArray l_x_array = cnpy::npz_load(m_events.toStdString(), "arr_1");
    cnpy::NpyArray l_y_array = cnpy::npz_load(m_events.toStdString(), "arr_2");
    cnpy::NpyArray l_polarities_array = cnpy::npz_load(m_events.toStdString(), "arr_3");
    size_t sizeLeftArray = l_timestamps_array.shape[0];

    auto *l_timestamps = l_timestamps_array.data<long>();
    auto *l_x = l_x_array.data<int16_t>();
    auto *l_y = l_y_array.data<int16_t>();
    auto *l_polarities = l_polarities_array.data<bool>();

    if (spinet.getNetworkConfig().NbCameras == 1) {
        long firstTimestamp = l_timestamps[0];
        long lastTimestamp = 0;
        auto event = Event();

        for (pass = 0; pass < static_cast<size_t>(m_nbPass); ++pass) {
            for (left = 0; left < sizeLeftArray; ++left) {
                event = Event(l_timestamps[left] + static_cast<long>(pass) * (lastTimestamp - firstTimestamp), l_x[left], l_y[left], l_polarities[left], 0);
                runSpikingNetwork(spinet, event, m_nbPass * left);
            }
            std::cout << "Finished iteration: " << pass + 1 << std::endl;
            lastTimestamp = static_cast<long>(l_timestamps[left-1]);
        }
    } else if (spinet.getNetworkConfig().NbCameras == 2) {
        cnpy::NpyArray r_timestamps_array = cnpy::npz_load(m_events.toStdString(), "arr_4");
        cnpy::NpyArray r_x_array = cnpy::npz_load(m_events.toStdString(), "arr_5");
        cnpy::NpyArray r_y_array = cnpy::npz_load(m_events.toStdString(), "arr_6");
        cnpy::NpyArray r_polarities_array = cnpy::npz_load(m_events.toStdString(), "arr_7");
        size_t sizeRightArray = r_timestamps_array.shape[0];

        auto *r_timestamps = r_timestamps_array.data<long>();
        auto *r_x = r_x_array.data<int16_t>();
        auto *r_y = r_y_array.data<int16_t>();
        auto *r_polarities = r_polarities_array.data<bool>();

        long firstLeftTimestamp = l_timestamps[0], firstRightTimestamp = r_timestamps[0], lastLeftTimestamp = 0, lastRightTimestamp = 0;
        auto event = Event();

        for (pass = 0; pass < static_cast<size_t>(m_nbPass); ++pass) {
            left = 0; right = 0;
            while (left < sizeLeftArray && right < sizeRightArray) {
                if (right >= sizeRightArray || l_timestamps[left] <= r_timestamps[right]) {
                    event = Event(l_timestamps[left] + static_cast<long>(pass) * (lastLeftTimestamp - firstLeftTimestamp), l_x[left], l_y[left], l_polarities[left], 0);
                    ++left;
                } else {
                    event = Event(r_timestamps[right] + static_cast<long>(pass) * (lastRightTimestamp - firstRightTimestamp), r_x[right], r_y[right], r_polarities[right], 1);
                    ++right;
                }
                runSpikingNetwork(spinet, event, m_nbPass * (sizeLeftArray + sizeRightArray));
            }
            std::cout << "Finished iteration: " << pass + 1 << std::endl;
            lastLeftTimestamp = static_cast<long>(l_timestamps[left-1]);
            lastRightTimestamp = static_cast<long>(r_timestamps[right-1]);
        }
    }
}

inline void NeuvisysThread::runSpikingNetwork(SpikingNetwork &spinet, Event &event, size_t sizeArray) {
    spinet.addEvent(event);
    if (event.camera() == 0) {
        if (m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[1] == 0 && m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2] == 0) {
            m_leftEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2-event.polarity()] = 255;
        }
    } else {
        if (m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[1] == 0 && m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2] == 0) {
            m_rightEventDisplay.at<cv::Vec3b>(event.y(), event.x())[2-event.polarity()] = 255;
        }
    }

//    if (count % Conf::EVENT_FREQUENCY == 0) {
//        spinet.updateNeurons(event.timestamp());
//    }

    std::chrono::duration<double> frameElapsed = std::chrono::high_resolution_clock::now() - m_frameTime;
    if (1000000 * frameElapsed.count() > m_precisionEvent) {
        m_frameTime = std::chrono::high_resolution_clock::now();
        display(spinet, sizeArray);
    }

    std::chrono::duration<double> trackingElapsed = std::chrono::high_resolution_clock::now() - m_trackingTime;
    if (1000000 * trackingElapsed.count() > m_precisionPotential) {
        m_trackingTime = std::chrono::high_resolution_clock::now();
        spinet.trackNeuron(event.timestamp(), m_idSimple, m_idComplex);
    }

    if (m_iterations % Conf::UPDATE_PARAMETER_FREQUENCY == 0) {
        spinet.updateNeuronsParameters(event.timestamp());
    }
    ++m_iterations;
}

inline void NeuvisysThread::display(SpikingNetwork &spinet, size_t sizeArray) {
    size_t count = 0;
    for (size_t i = 0; i < spinet.getNetworkConfig().L1XAnchor.size() * spinet.getNetworkConfig().L1Width; ++i) {
        for (size_t j = 0; j < spinet.getNetworkConfig().L1YAnchor.size() * spinet.getNetworkConfig().L1Height; ++j) {
            m_spikeTrain[count] = spinet.getSpikingNeuron(spinet.getLayout1(i, j, m_layer), 0);
            if (spinet.getNetworkConfig().SharingType == "none") {
                m_weightDisplay[count] = spinet.getWeightNeuron(spinet.getLayout1(i, j, m_layer), m_camera, m_synapse, 0, 0);
            }
            ++count;
        }
    } if (spinet.getNetworkConfig().SharingType == "patch") {
        count = 0;
        for (size_t wp = 0; wp < spinet.getNetworkConfig().L1XAnchor.size(); ++wp) {
            for (size_t hp = 0; hp < spinet.getNetworkConfig().L1XAnchor.size(); ++hp) {
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
    emit displayInformation(static_cast<int>(100 * m_iterations / sizeArray), 1000 * spinet.getNeuron(m_idSimple).getSpikingRate(), spinet.getNeuron(m_idSimple).getThreshold(), m_leftEventDisplay, m_rightEventDisplay, m_weightDisplay, spinet.getPotentialNeuron(m_idSimple, 0), m_spikeTrain);
    m_leftEventDisplay = 0;
    m_rightEventDisplay = 0;
}

void NeuvisysThread::onGuiInformation(const size_t index, const size_t layer, const size_t camera, const size_t synapse, const size_t precisionEvent, const size_t rangePotential, const size_t precisionPotential, const size_t rangeSpiketrain, const size_t precisionSpiketrain) {
    m_idSimple = index;
    m_layer = layer;
    m_camera = camera;
    m_synapse = synapse;

    m_precisionEvent = precisionEvent;
    m_rangePotential = rangePotential;
    m_precisionPotential = precisionPotential;
    m_rangeSpiketrain = rangeSpiketrain;
    m_precisionSpiketrain = precisionSpiketrain;
}
