#include "Neuron.hpp"

/* Defines an abstract neuron.
 * index: unique id.
 * layer: depth of the neuron in the spiking network.
 * m_conf: configuration file.
 * m_jitterSpeed: indicates the position of the neuron relative to the other neurons of the layer.
 * offset: removed to the position in order to access the correct weights of the neuron.
 */
Neuron::Neuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset) :
        m_index(index),
        m_layer(layer),
        m_conf(conf),
        m_pos(pos),
        m_offset(offset),
        m_trackingSpikeTrain(std::vector<size_t>(0)) {
    m_threshold = conf.VTHRESH;
    m_decay = 1.0;
    m_spike = false;
}

/* Returns the neuron's potential after the decay depending on the time passed from the last event.
 */
inline double Neuron::getPotential(size_t time) {
    return m_potential * exp(- static_cast<double>(time - m_timestampLastEvent) / m_conf.TAU_M);
}

inline double Neuron::potentialDecay(size_t time) {
    m_potential *= exp(- static_cast<double>(time) / m_conf.TAU_M);
}

inline double Neuron::refractoryPotential(size_t time) {
    return m_conf.DELTA_RP * exp(- static_cast<double>(time) / m_conf.TAU_RP);
}

inline double Neuron::adaptationPotentialDecay(size_t time) {
    m_adaptationPotential *= exp(- static_cast<double>(time) / m_conf.TAU_SRA);
}

/* Computes the neuron's lifespan as well as the exponential rolling average spiking rate depending on an alpha factor.
 */
void Neuron::updateState(size_t timeInterval, double alpha) {
    m_lifeSpan += timeInterval;
    double spikesPerSecond = static_cast<double>(m_spikeRateCounter) * (E6 / static_cast<double>(timeInterval)); // spikes/s
    m_spikeRateCounter = 0;
    m_spikingRateAverage = (alpha * spikesPerSecond) + (1.0 - alpha) * m_spikingRateAverage; // exponential rolling average
}

void Neuron::learningDecay(double count) {
    m_decay = 1 / (1 + m_conf.DECAY_RATE * count);
}

/* Rescales the neuron's threshold depending on the deviation from teh average and a target spike rate.
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

inline void Neuron::spikeRateAdaptation() {
    m_adaptationPotential += m_conf.DELTA_SRA;
}

inline bool Neuron::hasSpiked() {
    if (m_spike){
        m_spike = false;
        return true;
    }
    return false;
}

inline void Neuron::inhibition() {
    m_potential -= m_conf.ETA_INH;
}

void Neuron::saveState(std::string &fileName) {
    nlohmann::json state;

    writeJson(state);

    std::ofstream ofs(fileName + std::to_string(m_index) + ".json");
    if (ofs.is_open()) {
        ofs << std::setw(4) << state << std::endl;
    } else {
        std::cout << "cannot save neuron state file" << std::endl;
    }
    ofs.close();
}

void Neuron::loadState(std::string &fileName) {
    nlohmann::json state;
    std::ifstream ifs(fileName + std::to_string(m_index) + ".json");
    if (ifs.is_open()) {
        try {
            ifs >> state;
        } catch (const std::exception& e) {
            std::cerr << "In Neuron state file: " << fileName + ".json" << std::endl;
            throw;
        }
        readJson(state);
    } else {
        std::cout << "cannot open neuron state file" << std::endl;
    }
    ifs.close();
}

void Neuron::writeJson(nlohmann::json &state) {
    std::vector<size_t> position = {m_pos.x(), m_pos.y(), m_pos.z()};
    std::vector<size_t> offset = {m_offset.x(), m_offset.y(), m_offset.z()};
    std::vector<size_t> inIndex;
    std::vector<size_t> outIndex;
    std::vector<size_t> staticInhibitionIndex;
    std::vector<size_t> liInhibitionIndex;
    for (auto neuron : m_inConnections) {
        inIndex.push_back(neuron.get().getIndex());
    }
    for (auto neuron : m_outConnections) {
        outIndex.push_back(neuron.get().getIndex());
    }
    for (auto neuron : m_lateralStaticInhibitionConnections) {
        staticInhibitionIndex.push_back(neuron.get().getIndex());
    }
    for (auto neuron : m_lateralDynamicInhibitionConnections) {
        liInhibitionIndex.push_back(neuron.get().getIndex());
    }
    state["position"] = position;
    state["offset"] = offset;
    state["in_connections"] = inIndex;
    state["out_connections"] = outIndex;
    state["static_inhibition"] = staticInhibitionIndex;
    state["lateral_dynamic_inhibition"] = liInhibitionIndex;
    state["potential"] = m_potential;
    state["count_spike"] = m_totalSpike;
    state["threshold"] = m_threshold;
    state["lifespan"] = m_lifeSpan;
    state["spiking_rate"] = m_spikingRateAverage;
    state["learning_decay"] = m_decay;
    state["spike_train"] = m_trackingSpikeTrain;
    //    state["potential_train"] = m_trackingPotentialTrain;
}

void Neuron::readJson(const nlohmann::json &state) {
    m_totalSpike = state["count_spike"];
    m_threshold = state["threshold"];
    m_lifeSpan = state["lifespan"];
    m_decay = state["learning_decay"];
    m_spikingRateAverage = state["spiking_rate"];
//    m_potential = state["potential"];
}

void Neuron::trackPotential(const size_t time) {
    double potential = getPotential(time);
    m_trackingPotentialTrain.emplace_back(potential, time);
}

size_t Neuron::getActivityCount() {
    auto temp = m_activityCounter;
    m_activityCounter = 0;
    return temp;
}

void Neuron::addOutConnection(Neuron &neuron) {
    m_outConnections.emplace_back(neuron);
    m_topDownInhibitionWeights[neuron.getIndex()] = 0;
}

void Neuron::addInConnection(Neuron &neuron) {
    m_inConnections.emplace_back(neuron);
}

void Neuron::addLateralStaticInhibitionConnections(Neuron &neuron) {
    m_lateralStaticInhibitionConnections.emplace_back(neuron);
}

void Neuron::addLateralDynamicInhibitionConnections(Neuron &neuron) {
    m_lateralDynamicInhibitionConnections.emplace_back(neuron);
    m_lateralInhibitionWeights[neuron.getIndex()] = 0;
}
