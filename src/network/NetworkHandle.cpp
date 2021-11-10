//
// Created by alphat on 14/04/2021.
//

#include "NetworkHandle.hpp"

/* Creates the spiking neural network from the SpikingNetwork class.
 * Loads the configuration files locally.
 * Loads possible network weights if the network has already been created and saved before.
 */
NetworkHandle::NetworkHandle(const std::string &networkPath) : m_spinet(networkPath), m_networkConf(NetworkConfig(networkPath)),
                                                               m_simpleNeuronConf(m_networkConf.getNetworkPath() + "configs/simple_cell_config.json", 0),
                                                               m_complexNeuronConf(m_networkConf.getNetworkPath() + "configs/complex_cell_config.json", 1),
                                                               m_criticNeuronConf(m_networkConf.getNetworkPath() + "configs/critic_cell_config.json", 2),
                                                               m_actorNeuronConf(m_networkConf.getNetworkPath() + "configs/actor_cell_config.json",3) {
    m_spinet.loadWeights();
}

/* Launches the spiking network on the specified event file 'nbPass' times.
 * The config file of the network must precise if the event file is stereo or mono.
 */
void NetworkHandle::multiplePass(const std::string &events, size_t nbPass) {
    auto eventPacket = std::vector<Event>();
    if (m_networkConf.getNbCameras() == 1) {
        eventPacket = mono(events, nbPass);
    } else if (m_networkConf.getNbCameras() == 2) {
        eventPacket = stereo(events, nbPass);
    }

    long time = eventPacket.front().timestamp();

    for (auto event : eventPacket) {
        m_spinet.runEvent(event);

        if (event.timestamp() - time > UPDATE_INTERVAL / E6) {
            time = event.timestamp();
            updateNeuronStates(UPDATE_INTERVAL);
        }
    }

    save(nbPass, events);
}

/* Saves information about the network actual state.
 * Number of times the network has been launched and the name of the event file use can be precised.
 */
void NetworkHandle::save(const size_t nbRun = 0, const std::string &eventFileName = "") {
    std::cout << "Saving Network..." << std::endl;
    std::string fileName;
    fileName = m_networkConf.getNetworkPath() + "networkState";

    json state;
    state["event_file_name"] = eventFileName;
    state["nb_run"] = nbRun;
    state["learning_data"] = m_saveData;
    state["action_rate"] = m_networkConf.getActionRate();
    state["exploration_factor"] = m_networkConf.getExplorationFactor();

    std::ofstream ofs(fileName + ".json");
    if (ofs.is_open()) {
        ofs << std::setw(4) << state << std::endl;
    } else {
        std::cout << "cannot save network state file" << std::endl;
    }
    ofs.close();

    m_spinet.saveNetwork();

    std::cout << "Finished." << std::endl;
}

/* Opens an event file (in the npz format) and load all the events in memory into a vector.
 * If nbPass is greater than 1, the events are concatenated multiple times and the timestamps are updated in accordance.
 * Returns the vector of events.
 * Only for mono camera event files.
 */
std::vector<Event> NetworkHandle::mono(const std::string &events, size_t nbPass = 1) {
    std::vector<Event> eventPacket;
    size_t pass, count;

    cnpy::NpyArray timestamps_array = cnpy::npz_load(events, "arr_0");
    cnpy::NpyArray x_array = cnpy::npz_load(events, "arr_1");
    cnpy::NpyArray y_array = cnpy::npz_load(events, "arr_2");
    cnpy::NpyArray polarities_array = cnpy::npz_load(events, "arr_3");
    size_t sizeLeftArray = timestamps_array.shape[0];

    auto *l_timestamps = timestamps_array.data<long>();
    auto *l_x = x_array.data<int16_t>();
    auto *l_y = y_array.data<int16_t>();
    auto *l_polarities = polarities_array.data<bool>();

    long firstTimestamp = l_timestamps[0];
    long lastTimestamp = static_cast<long>(l_timestamps[sizeLeftArray - 1]);
    Event event{};

    for (pass = 0; pass < static_cast<size_t>(nbPass); ++pass) {
        for (count = 0; count < sizeLeftArray; ++count) {
            event = Event(l_timestamps[count] + static_cast<long>(pass) * (lastTimestamp - firstTimestamp), l_x[count], l_y[count],
                          l_polarities[count], 0);
            eventPacket.push_back(event);
        }
    }
    return eventPacket;
}

/* Same as mono function but for stereo camera event files.
 */
std::vector<Event> NetworkHandle::stereo(const std::string &events, size_t nbPass) {
    std::vector<Event> eventPacket;
    size_t pass, left, right;

    cnpy::NpyArray l_timestamps_array = cnpy::npz_load(events, "arr_0");
    cnpy::NpyArray l_x_array = cnpy::npz_load(events, "arr_1");
    cnpy::NpyArray l_y_array = cnpy::npz_load(events, "arr_2");
    cnpy::NpyArray l_polarities_array = cnpy::npz_load(events, "arr_3");
    size_t sizeLeftArray = l_timestamps_array.shape[0];

    auto *l_timestamps = l_timestamps_array.data<long>();
    auto *l_x = l_x_array.data<int16_t>();
    auto *l_y = l_y_array.data<int16_t>();
    auto *l_polarities = l_polarities_array.data<bool>();

    cnpy::NpyArray r_timestamps_array = cnpy::npz_load(events, "arr_4");
    cnpy::NpyArray r_x_array = cnpy::npz_load(events, "arr_5");
    cnpy::NpyArray r_y_array = cnpy::npz_load(events, "arr_6");
    cnpy::NpyArray r_polarities_array = cnpy::npz_load(events, "arr_7");
    size_t sizeRightArray = r_timestamps_array.shape[0];

    auto *r_timestamps = r_timestamps_array.data<long>();
    auto *r_x = r_x_array.data<int16_t>();
    auto *r_y = r_y_array.data<int16_t>();
    auto *r_polarities = r_polarities_array.data<bool>();

    long firstLeftTimestamp = l_timestamps[0], firstRightTimestamp = r_timestamps[0], lastLeftTimestamp = static_cast<long>(l_timestamps[
            sizeLeftArray - 1]), lastRightTimestamp = static_cast<long>(r_timestamps[sizeRightArray - 1]);
    long l_t, r_t;
    Event event{};

    for (pass = 0; pass < static_cast<size_t>(nbPass); ++pass) {
        left = 0;
        right = 0;
        while (left < sizeLeftArray && right < sizeRightArray) {
            l_t = l_timestamps[left] + static_cast<long>(pass) * (lastLeftTimestamp - firstLeftTimestamp);
            r_t = r_timestamps[right] + static_cast<long>(pass) * (lastRightTimestamp - firstRightTimestamp);
            if (right >= sizeRightArray || l_t <= r_t) {
                event = Event(l_t / E3, l_x[left], l_y[left], l_polarities[left], 0);
                ++left;
            } else if (left >= sizeLeftArray || l_t > r_t) {
                event = Event(r_t / E3, r_x[right], r_y[right], r_polarities[right], 1);
                ++right;
            }
            eventPacket.push_back(event);
        }
    }
    return eventPacket;
}

void NetworkHandle::updateActor(long timestamp, size_t actor) {
    auto neuron = m_spinet.getNeuron(actor, m_spinet.getNetworkStructure().size() - 1);
    neuron.get().spike(timestamp);

    auto V = valueFunction(static_cast<double>(timestamp));
    double VDot = valueDerivative(m_saveData["value"]);
    auto tdError = VDot - V / m_networkConf.getTauR() + m_reward;

    neuron.get().setNeuromodulator(tdError);
    neuron.get().weightUpdate(); // TODO: what about the eligibility traces (previous action) ?
    m_spinet.normalizeActions();
}

double NetworkHandle::getScore(long time) {
    double mean = 0;
    for (auto it = m_saveData["reward"].end(); it != m_saveData["reward"].end() - time && it != m_saveData["reward"].begin(); --it) {
        mean += *it;
    }
    mean /= static_cast<double>(time);
    m_saveData["score"].push_back(mean);
    return mean;
}

void NetworkHandle::storeLearningMetrics(double time, size_t nbEvents) {
    m_saveData["nbEvents"].push_back(static_cast<double>(nbEvents));

    auto V = valueFunction(time);
    double VDot = valueDerivative(m_saveData["value"]);
    auto tdError = VDot - V / m_networkConf.getTauR() + m_reward;

    m_saveData["reward"].push_back(m_reward);
    m_saveData["value"].push_back(V);
    m_saveData["valueDot"].push_back(VDot);
    m_saveData["tdError"].push_back(tdError);
}

double NetworkHandle::valueFunction(double time) {
    double value = 0;
    for (size_t i = 0; i < m_spinet.getNetworkStructure()[2]; ++i) {
        value += m_spinet.getNeuron(i, 2).get().updateKernelSpikingRate(time);
    }
    return m_networkConf.getNu() * value / static_cast<double>(m_spinet.getNetworkStructure()[2]) + m_networkConf.getV0();
}

double NetworkHandle::valueDerivative(const std::vector<double> &value) {
    int nbPreviousTD = static_cast<int>(m_networkConf.getActionRate() / E3) / DT;
    if (value.size() > nbPreviousTD) {
        return 50 * Util::secondOrderNumericalDifferentiationMean(m_saveData["value"], nbPreviousTD);
    } else {
        return 0;
    }
}

void NetworkHandle::transmitReward(const double reward) {
    m_reward = reward;
    m_spinet.transmitNeuromodulator(reward);
}

void NetworkHandle::transmitEvents(const std::vector<Event> &eventPacket) {
    storeLearningMetrics(static_cast<double>(eventPacket.back().timestamp()), eventPacket.size());

    for (auto event : eventPacket) {
        m_spinet.runEvent(event);
    }
}

std::vector<uint64_t> NetworkHandle::resolveMotor() {
    std::vector<uint64_t> motorActivations(m_spinet.getNetworkStructure().back(), 0);

    size_t layer = m_spinet.getNetworkStructure().size() - 1;
    for (size_t i = 0; i < m_spinet.getNetworkStructure().back(); ++i) {
        motorActivations[m_spinet.getNeuron(i, layer).get().getIndex()] = m_spinet.getNeuron(i, layer).get().getActivityCount();
    }
    return motorActivations;
}

void NetworkHandle::learningDecay(size_t iteration) {
    double decay = 1 + getNetworkConfig().getDecayRate() * static_cast<double>(iteration);

    m_spinet.getNeuron(0, 2).get().learningDecay(decay);
    m_spinet.getNeuron(0, 3).get().learningDecay(decay);

    m_networkConf.setExplorationFactor(m_networkConf.getExplorationFactor() / decay);
    if (m_networkConf.getActionRate() > m_networkConf.getMinActionRate()) {
        m_networkConf.setActionRate(static_cast<long>(m_networkConf.getActionRate() / decay));
    }
}

void NetworkHandle::trackNeuron(const long time, const size_t id, const size_t layer) {
    if (m_simpleNeuronConf.TRACKING == "partial") {
        if (m_spinet.getNetworkStructure()[layer] > 0) {
            m_spinet.getNeuron(id, layer).get().trackPotential(time);
        }
    }
}

cv::Mat NetworkHandle::getWeightNeuron(size_t idNeuron, size_t layer, size_t camera, size_t synapse, size_t z) {
    if (m_spinet.getNetworkStructure()[layer] > 0) {
        auto dim = m_spinet.getNeuron(0, layer).get().getWeightsDimension();
        cv::Mat weightImage = cv::Mat::zeros(static_cast<int>(dim[1]), static_cast<int>(dim[0]), CV_8UC3);

        double weight;
        for (size_t x = 0; x < dim[0]; ++x) {
            for (size_t y = 0; y < dim[1]; ++y) {
                if (layer == 0) {
                    for (size_t p = 0; p < NBPOLARITY; p++) {
                        weight = m_spinet.getNeuron(idNeuron, layer).get().getWeights(p, camera, synapse, x, y) * 255;
                        weightImage.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[static_cast<int>(2 - p)] = static_cast<unsigned char>
                                (weight);
                    }
                } else {
                    weight = m_spinet.getNeuron(idNeuron, layer).get().getWeights(x, y, z) * 255;
                    weightImage.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[0] = static_cast<unsigned char>(weight);
                }
            }
        }
        double min, max;
        minMaxIdx(weightImage, &min, &max);
        cv::Mat weights = weightImage * 255 / max;
        return weights;
    }
    return cv::Mat::zeros(0, 0, CV_8UC3);
}

cv::Mat NetworkHandle::getSummedWeightNeuron(size_t idNeuron, size_t layer) {
    if (m_spinet.getNetworkStructure()[layer] > 0) {
        return m_spinet.getNeuron(idNeuron, layer).get().summedWeightMatrix();
    }
    return cv::Mat::zeros(0, 0, CV_8UC3);
}

void NetworkHandle::updateNeuronStates(long timeInterval) {
    m_spinet.updateNeuronsStates(timeInterval);
}
