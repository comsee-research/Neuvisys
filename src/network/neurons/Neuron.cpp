//
// Created by Thomas on 14/04/2021.
//

#include "Neuron.hpp"

/**
 * Defines an abstract neuron.
 * @param index - unique id.
 * @param layer - depth of the neuron in the spiking network.
 * @param conf - configuration file.
 * @param pos - indicates the position of the neuron relative to the other neurons of the layer.
 * @param offset - removed to the position in order to access the correct weights of the neuron.
 */
Neuron::Neuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset) :
        m_index(index),
        m_layer(layer),
        m_conf(conf),
        m_pos(pos),
        m_offset(offset),
        m_range_x(1),
        m_range_y(1),
        m_trackingSpikeTrain(std::vector<size_t>(0)),
        m_amount_of_events(std::vector<size_t>(4, 0)) {
    // m_sumOfInhibWeights(std::vector<std::vector<double>(8,0)>(3))
    m_threshold = conf.VTHRESH;
    m_decay = 1.0;
    m_spike = false;
    m_adaptationPotential = 0;
}

/**
 * Returns the neuron's potential after the decay depending on the time passed from the last event.
 * @param time
 * @return
 */
inline double Neuron::getPotential(size_t time) {
    return m_potential * exp(-static_cast<double>(time - m_timestampLastEvent) / m_conf.TAU_M);
}

/**
 *
 * @param time
 * @return
 */
inline double Neuron::refractoryPotential(size_t time) {
    return m_conf.DELTA_RP * exp(-static_cast<double>(time - m_spikingTime) / m_conf.TAU_RP);
}

/**
 *
 * @param time
 */
inline void Neuron::potentialDecay(size_t time) {
    m_potential *= exp(-static_cast<double>(time - m_timestampLastEvent) / m_conf.TAU_M);
}

/**
 *
 * @param time
 */
inline void Neuron::adaptationPotentialDecay(size_t time) {
    m_adaptationPotential *= exp(-static_cast<double>(time - m_timestampLastEvent) / m_conf.TAU_SRA);
}

/**
 *
 * @param potential
 */
void Neuron::assignToPotentialTrain(std::pair<double, uint64_t> potential) {
    m_trackingPotentialTrain.push_back(potential);
}

/**
 *
 */
void Neuron::assignToPotentialThreshold() {
    m_potentialThreshold.push_back(m_threshold);
}

/**
 *
 * @param type
 */
void Neuron::assignToAmountOfEvents(int type) {
    if (m_amount_of_events.size() == 0) {
        for (int i = 0; i < 4; i++) {
            m_amount_of_events.push_back(0);
        }
    }
    m_amount_of_events.at(type) += 1;
}

/**
 *
 * @param type
 * @param pos
 * @param wi
 */
void Neuron::assignToSumInhibWeights(int type, Position pos, double wi) {
    if (m_sumOfInhibWeights.size() == 0) {
        for (int i = 0; i < 2; i++) {
            std::vector<double> temp;
            m_sumOfInhibWeights.push_back(temp);

            if (m_range_x == 0) {
                for (int j = -m_range_y; j < m_range_y; j++) {
                    m_sumOfInhibWeights.at(i).push_back(0);
                }
            } else if (m_range_y == 0) {
                for (int j = -m_range_x; j < m_range_x; j++) {
                    m_sumOfInhibWeights.at(i).push_back(0);
                }
            } else {
                for (int j = -m_range_x; j < m_range_x + 1; j++) {
                    for (int k = -m_range_y; k < m_range_y + 1; k++) {
                        if (j == 0 && k == 0) {
                            continue;
                        } else {
                            m_sumOfInhibWeights.at(i).push_back(0);
                        }
                    }
                }
            }
        }
    }
    int x_border_min = m_pos.x() - m_range_x;
    int y_border_min = m_pos.y() - m_range_y;
    int position;
    if ((pos.x() > m_pos.x()) || (pos.x() == m_pos.x() && pos.y() > m_pos.y())) {
        position = (2 * (m_range_y) + 1) * (pos.x() - x_border_min) + (pos.y() - y_border_min) - 1;
    } else {
        position = (2 * (m_range_y) + 1) * (pos.x() - x_border_min) + (pos.y() - y_border_min);
    }
    if (m_sumOfInhibWeights.at(type).size() <= position) {
        std::cout << "sizes = " << m_sumOfInhibWeights.at(type).size() << " ; position = " << position << std::endl;
        std::cout << "pos x = " << pos.x() << " ; x border min = " << x_border_min << " ; pos y = " << pos.y() << " ; y_border_min = " << y_border_min
                  << std::endl;
        std::cout << "m pos x = " << m_pos.x() << " ; m pos y = " << m_pos.y() << std::endl;
        std::cout << "m_range_x = " << m_range_x << " ; m_range_y = " << m_range_y << std::endl;
    }
    m_sumOfInhibWeights.at(type).at(position) += wi;
}

/**
 *
 * @param type
 * @param variation
 */
void Neuron::assignToTimingOfInhibition(int type, std::tuple<double, double, uint64_t> variation) {
    if (m_timingOfInhibition.size() == 0) {
        std::vector<std::tuple<double, double, uint64_t>> temp;
        for (int i = 0; i < 3; i++) {
            m_timingOfInhibition.push_back(temp);
        }
    }
    m_timingOfInhibition.at(type).push_back(variation);
}

/**
 *
 * @return
 */
std::vector<std::pair<double, uint64_t>> Neuron::getPotentialTrain() {
    return m_trackingPotentialTrain;
}

/**
 *
 * @return
 */
std::vector<double> Neuron::getPotentialThreshold() {
    return m_potentialThreshold;
}

/**
 *
 * @return
 */
std::vector<size_t> Neuron::getAmountOfEvents() {
    return m_amount_of_events;
}

/**
 *
 * @return
 */
std::vector<std::vector<double>> Neuron::getSumInhibWeights() {
    return m_sumOfInhibWeights;
}

/**
 *
 * @return
 */
std::vector<std::vector<std::tuple<double, double, uint64_t>>> Neuron::getTimingOfInhibition() {
    return m_timingOfInhibition;
}

/**
* Computes the neuron's lifespan as well as the average spiking rate.
* @param timeInterval
*/
void Neuron::updateState(size_t timeInterval) {
    m_lifeSpan += timeInterval;
    double spikesPerSecond = static_cast<double>(m_spikeRateCounter) * (E6 / static_cast<double>(timeInterval)); // spikes/s
    auto alpha = 0.1;
    m_spikingRateAverage = (alpha * spikesPerSecond) + (1.0 - alpha) * m_spikingRateAverage; // exponential rolling average
    m_spikeRateCounter = 0;
}

/**
 *
 * @param count
 */
void Neuron::learningDecay(double count) {
    m_decay = 1 / (1 + m_conf.DECAY_RATE * count);
}

/**
 * Rescales the neuron's threshold depending on the deviation from the average spike rate and a target spike rate.
 */
inline void Neuron::thresholdAdaptation() {
    if (m_spikingRateAverage > m_conf.TARGET_SPIKE_RATE) {
        m_threshold += m_conf.ETA_SR * (1 - exp(m_conf.TARGET_SPIKE_RATE - m_spikingRateAverage));
    } else {
        m_threshold -= m_conf.ETA_SR * (1 - exp(m_spikingRateAverage - m_conf.TARGET_SPIKE_RATE));
    }

    if (m_threshold < m_conf.MIN_THRESH) {
        m_threshold = m_conf.MIN_THRESH;
    }
}

/**
 *
 */
inline void Neuron::spikeRateAdaptation() {
    m_adaptationPotential += m_conf.DELTA_SRA;
}

/**
 *
 * @return
 */
inline bool Neuron::hasSpiked() {
    if (m_spike) {
        m_spike = false;
        return true;
    }
    return false;
}

/**
 *
 * @param event
 */
inline void Neuron::newStaticInhibitoryEvent(NeuronEvent event) {
    potentialDecay(event.timestamp());
    adaptationPotentialDecay(event.timestamp());
    m_potential -= m_conf.ETA_INH;
    m_timestampLastEvent = event.timestamp();
}

/**
 *
 * @param filePath
 */
void Neuron::saveState(std::string &filePath) {
    nlohmann::json state;

    writeJson(state);

    std::ofstream ofs(filePath + std::to_string(m_index) + ".json");
    if (ofs.is_open()) {
        ofs << std::setw(4) << state << std::endl;
    } else {
        std::cout << "cannot save neuron state file" << std::endl;
    }
    ofs.close();
}

/**
 *
 * @param filePath
 */
void Neuron::loadState(std::string &filePath) {
    nlohmann::json state;
    std::ifstream ifs(filePath + std::to_string(m_index) + ".json");
    if (ifs.is_open()) {
        try {
            ifs >> state;
        } catch (const std::exception &e) {
            std::cerr << "In Neuron state file: " << filePath + ".json" << std::endl;
            throw;
        }
        readJson(state);
    } else {
        std::cout << "cannot open neuron state file" << std::endl;
    }
    ifs.close();
}

/**
 *
 * @param state
 */
void Neuron::writeJson(nlohmann::json &state) {
    std::vector<size_t> position = {m_pos.x(), m_pos.y(), m_pos.z()};
    std::vector<size_t> offset = {m_offset.x(), m_offset.y(), m_offset.z()};
    std::vector<size_t> inIndex;
    std::vector<size_t> outIndex;
    std::vector<size_t> staticInhibitionIndex;
    std::vector<size_t> tdInhibitionIndex;
    std::vector<size_t> liInhibitionIndex;
    for (auto neuron: m_inConnections) {
        inIndex.push_back(neuron.get().getIndex());
    }
    for (auto neuron: m_outConnections) {
        outIndex.push_back(neuron.get().getIndex());
    }
    for (auto neuron: m_lateralStaticInhibitionConnections) {
        staticInhibitionIndex.push_back(neuron.get().getIndex());
    }
    for (auto neuron: m_topDownDynamicInhibitionConnections) {
        tdInhibitionIndex.push_back(neuron.get().getIndex());
    }
    for (auto neuron: m_lateralDynamicInhibitionConnections) {
        liInhibitionIndex.push_back(neuron.get().getIndex());
    }
    state["position"] = position;
    state["offset"] = offset;
    state["in_connections"] = inIndex;
    state["out_connections"] = outIndex;
    state["static_inhibition"] = staticInhibitionIndex;
    state["topdown_dynamic_inhibition"] = tdInhibitionIndex;
    state["lateral_dynamic_inhibition"] = liInhibitionIndex;
    state["potential"] = m_potential;
    state["count_spike"] = m_totalSpike;
    state["threshold"] = m_threshold;
    state["lifespan"] = m_lifeSpan;
    state["spiking_rate"] = m_spikingRateAverage;
    state["learning_decay"] = m_decay;
    state["spike_train"] = m_trackingSpikeTrain;
}

/**
 *
 * @param state
 */
void Neuron::readJson(const nlohmann::json &state) {
    m_totalSpike = state["count_spike"];
//    m_threshold = state["threshold"];
//    m_lifeSpan = state["lifespan"];
    m_decay = state["learning_decay"];
    m_spikingRateAverage = state["spiking_rate"];
//    m_potential = state["potential"];
}

/**
 *
 * @param time
 */
void Neuron::trackPotential(const size_t time) {
    double potential = getPotential(time);
    m_trackingPotentialTrain.emplace_back(potential, time);
}

/**
 *
 * @return
 */
size_t Neuron::getActivityCount() {
    return m_activityCounter;
}

/**
 *
 */
void Neuron::resetActivityCount() {
    m_activityCounter = 0;
}

/**
 *
 * @param neuron
 */
void Neuron::addOutConnection(Neuron &neuron) {
    m_outConnections.emplace_back(neuron);
}

/**
 *
 * @param neuron
 */
void Neuron::addInConnection(Neuron &neuron) {
    m_inConnections.emplace_back(neuron);
}

/**
 *
 * @param neuron
 */
void Neuron::addTopDownDynamicInhibitionConnection(Neuron &neuron) {
    m_topDownDynamicInhibitionConnections.emplace_back(neuron);
}

/**
 *
 * @param neuron
 */
void Neuron::initializeTopDownDynamicInhibitionWeights() {
    m_topDownInhibitionWeights.addNewWeight();
}

/**
 *
 * @param neuron
 */
void Neuron::addLateralStaticInhibitionConnections(Neuron &neuron) {
    m_lateralStaticInhibitionConnections.emplace_back(neuron);
}

/**
 *
 * @param neuron
 */
void Neuron::addLateralDynamicInhibitionConnections(Neuron &neuron) {
    m_lateralDynamicInhibitionConnections.emplace_back(neuron);
    m_lateralInhibitionWeights.addNewWeight();
}

void Neuron::resetNeuron() {
    m_potential = 0;
    m_timestampLastEvent = 0;
    m_lastSpikingTime = 0;
    m_spikingTime = 0;
    m_totalSpike = 0;
    m_spikeRateCounter = 0;
    m_activityCounter = 0;
    m_decay = 1.0;
    m_adaptationPotential = 0;
    m_threshold = m_conf.VTHRESH;
    m_spike = false;
    m_lifeSpan = 0;
    m_spikingRateAverage = 0;
    m_trackingSpikeTrain.clear();
    m_trackingPotentialTrain.clear();
    m_potentialThreshold.clear();
    m_amount_of_events.clear();
    for (int i = 0; i < m_sumOfInhibWeights.size(); i++) {
        m_sumOfInhibWeights[i].clear();
    }
    m_sumOfInhibWeights.clear();
    for (int j = 0; j < m_timingOfInhibition.size(); j++) {
        m_timingOfInhibition[j].clear();
    }
    m_timingOfInhibition.clear();
}