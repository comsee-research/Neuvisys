//
// Created by alphat on 14/04/2021.
//

#include "NetworkHandle.hpp"

NetworkHandle::NetworkHandle(const std::string &networkPath) : m_spinet(networkPath), m_conf(NetworkConfig(networkPath)),
                                                               m_simpleNeuronConf(m_conf.getNetworkPath() + "configs/simple_cell_config.json", 0),
                                                               m_complexNeuronConf(m_conf.getNetworkPath() + "configs/complex_cell_config.json", 1),
                                                               m_criticNeuronConf(m_conf.getNetworkPath() + "configs/critic_cell_config.json", 2),
                                                               m_actorNeuronConf(m_conf.getNetworkPath() + "configs/actor_cell_config.json", 3) {
    m_spinet.loadWeights();
}

void NetworkHandle::multiplePass(const std::string &events, size_t nbPass) {
    auto eventPacket = std::vector<Event>();
    if (m_conf.getNbCameras() == 1) {
        eventPacket = mono(events, nbPass);
    } else if (m_conf.getNbCameras() == 2) {
        eventPacket = stereo(events, nbPass);
    }

    for (auto event : eventPacket) {
        m_spinet.runEvent(event);
    }

    save(nbPass, events);
}

void NetworkHandle::updateActor(long timestamp, size_t actor) {
    auto neuron = getNeuron(actor, getNetworkStructure().size() - 1);
    neuron.get().spike(timestamp);

    auto meanDValues = 50 * Util::secondOrderNumericalDifferentiationMean(m_saveData["value"].end() - 50, m_saveData["value"].end());

    neuron.get().setNeuromodulator(meanDValues);
    neuron.get().weightUpdate(); // TODO: what about the eligibility traces (previous action) ?
    m_spinet.normalizeActions();
    //    std::this_thread::sleep_for(2s);
}

double NetworkHandle::getScore(long time) {
    double mean = 0;
    if (m_saveData["reward"].size() > time) {
        for (auto it = m_saveData["reward"].end(); it != m_saveData["reward"].end() - time && it != m_saveData["reward"].begin(); --it) {
            mean += *it;
        }
        mean /= static_cast<double>(time);
        m_saveData["score"].push_back(mean);
        return mean;
    }
    return 0;
}

double NetworkHandle::storeLearningMetrics(const double time, const size_t nbEvents) {
    m_saveData["nbEvents"].push_back(static_cast<double>(nbEvents));

    double value = 0;
    for (const auto &critic: m_spinet.getNeurons()[2]) {
        value += critic.get().updateKernelSpikingRate(time);
    }
    auto V = m_conf.getNu() * value / static_cast<double>(getNetworkStructure()[2]) +
             m_conf.getV0();
    //    auto VDot = m_conf.getNu() * valueDerivative / static_cast<double>(m_neurons[2].size());
    auto tdError = -V / m_conf.getTauR() + m_reward;

    m_saveData["reward"].push_back(m_reward);
    m_saveData["value"].push_back(V);

    size_t nbPreviousTD = (m_conf.getActionRate() / Conf::E3) / 1;
    if (m_saveData["valueDot"].size() > nbPreviousTD) {
        auto meanDValues = 500 * Util::secondOrderNumericalDifferentiationMean(m_saveData["value"].end() - nbPreviousTD, m_saveData["value"]
                .end());
        m_saveData["valueDot"].push_back(meanDValues);
    } else {
        m_saveData["valueDot"].push_back(0);
    }
    m_saveData["tdError"].push_back(tdError);
    if (time < 2 * Conf::E6) {
        return 0;
    } else {
        return tdError;
    }
}

void NetworkHandle::transmitReward(const double reward) {
    m_spinet.transmitNeuromodulator(reward);
}

void NetworkHandle::transmitEvents(const std::vector<Event> &eventPacket) {
    storeLearningMetrics(static_cast<double>(eventPacket.back().timestamp()), eventPacket.size());

    for (auto event : eventPacket) {
        m_spinet.runEvent(event);
    }
}

std::vector<uint64_t> NetworkHandle::resolveMotor() {
    std::vector<uint64_t> motorActivations(m_spinet.getNeurons()[m_spinet.getNeurons().size() - 1].size(), 0);
    for (auto &neuron: m_spinet.getNeurons()[m_spinet.getNeurons().size() - 1]) {
        motorActivations[neuron.get().getIndex()] = neuron.get().getSpikeCount();
        neuron.get().resetSpikeCounter();
    }
    return motorActivations;
}

void NetworkHandle::learningDecay(size_t iteration) {
//    m_criticNeruonConf().ETA = .m_criticNeruonConf().ETA / (1 + getNetworkConfig().getDecayRate() * static_cast<double>(iteration));
//    getActorNeuronConfig().ETA = getActorNeuronConfig().ETA / (1 + getNetworkConfig().getDecayRate() * static_cast<double>(iteration));

    if (m_criticNeuronConf.TAU_K > m_criticNeuronConf.MIN_TAU_K) {
//        m_spinet.m_criticNeruonConf().TAU_K = m_spinet.m_criticNeruonConf().TAU_K / (1 + getNetworkConfig().getDecayRate() * static_cast<double>(iteration));
    }
    if (m_criticNeuronConf.NU_K > m_criticNeuronConf.MIN_NU_K) {
//        m_spinet.m_criticNeruonConf().NU_K = m_spinet.m_criticNeruonConf().NU_K / (1 + getNetworkConfig().getDecayRate() * static_cast<double>(iteration));
    }

    m_conf.setExplorationFactor(
            getNetworkConfig().getExplorationFactor() / (1 + getNetworkConfig().getDecayRate() * static_cast<double>
            (iteration)));

    if (getNetworkConfig().getActionRate() > getNetworkConfig().getMinActionRate()) {
        m_conf.setActionRate(
                static_cast<long>(getNetworkConfig().getActionRate() / (1 + getNetworkConfig().getDecayRate() * static_cast<double>(iteration))));
    }
}

void NetworkHandle::save(const size_t nbRun, const std::string &eventFileName) {
    std::cout << "Saving Network..." << std::endl;
    std::string fileName;
    fileName = m_conf.getNetworkPath() + "networkState";

    json state;
    state["event_file_name"] = eventFileName;
    state["nb_run"] = nbRun;
    state["learning_data"] = m_saveData;
    state["action_rate"] = m_conf.getActionRate();
    state["exploration_factor"] = m_conf.getExplorationFactor();

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

void NetworkHandle::trackNeuron(const long time, const size_t id, const size_t layer) {
    if (m_simpleNeuronConf.TRACKING == "partial") {
        if (!m_spinet.getNeurons()[layer].empty()) {
            m_spinet.getNeurons()[layer][id].get().trackPotential(time);
        }
    }
}

std::reference_wrapper<Neuron> &NetworkHandle::getNeuron(const size_t index, const size_t layer) {
    if (layer < m_spinet.getNeurons().size()) {
        if (index < m_spinet.getNeurons()[layer].size()) {
            return m_spinet.getNeurons()[layer][index];
        }
    }
    throw std::runtime_error("Wrong layer or index for neuron selection");
}

std::vector<size_t> NetworkHandle::getNetworkStructure() {
    std::vector<size_t> structure;
    for (auto &m_neuron : m_spinet.getNeurons()) {
        structure.push_back(m_neuron.size());
    }
    return structure;
}

cv::Mat NetworkHandle::getWeightNeuron(size_t idNeuron, size_t layer, size_t camera, size_t synapse, size_t z) {
    if (!m_spinet.getNeurons()[layer].empty()) {
        auto dim = m_spinet.getNeurons()[layer][0].get().getWeightsDimension();
        cv::Mat weightImage = cv::Mat::zeros(static_cast<int>(dim[1]), static_cast<int>(dim[0]), CV_8UC3);

        double weight;
        for (size_t x = 0; x < dim[0]; ++x) {
            for (size_t y = 0; y < dim[1]; ++y) {
                if (layer == 0) {
                    for (size_t p = 0; p < NBPOLARITY; p++) {
                        weight = m_spinet.getNeurons()[layer][idNeuron].get().getWeights(p, camera, synapse, x, y) * 255;
                        weightImage.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[static_cast<int>(2 - p)] = static_cast<unsigned char>
                                (weight);
                    }
                } else {
                    weight = m_spinet.getNeurons()[layer][idNeuron].get().getWeights(x, y, z) * 255;
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
    if (!m_spinet.getNeurons()[layer].empty()) {
        return m_spinet.getNeurons()[layer][idNeuron].get().summedWeightMatrix();
    }
    return cv::Mat::zeros(0, 0, CV_8UC3);
}

std::vector<Event> mono(const std::string &events, size_t nbPass) {
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

std::vector<Event> stereo(const std::string &events, size_t nbPass) {
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
                event = Event(l_t / 1000, l_x[left], l_y[left], l_polarities[left], 0);
                ++left;
            } else if (left >= sizeLeftArray || l_t > r_t) {
                event = Event(r_t / 1000, r_x[right], r_y[right], r_polarities[right], 1);
                ++right;
            }
            eventPacket.push_back(event);
        }
    }
    return eventPacket;
}