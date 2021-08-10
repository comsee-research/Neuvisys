#include "Neuron.hpp"

using json = nlohmann::json;

Neuron::Neuron(size_t index, NeuronConfig &conf, Position pos, Position offset) :
        m_index(index),
        conf(conf),
        m_pos(pos),
        m_offset(offset),
        m_recentSpikes(std::list<size_t>(Conf::TIME_WINDOW_SR)),
        m_trackingThresholds(std::vector<double>(0)),
        m_trackingSpikeTrain(std::vector<long>(0)) {
    m_threshold = conf.VTHRESH;
    m_learningDecay = 1.0;
    m_spike = false;
}

inline double Neuron::getPotential(const long time) {
    return m_potential * exp(- static_cast<double>(time - m_timestampLastEvent) / conf.TAU_M);
}

inline double Neuron::potentialDecay(const long time) {
    m_potential *= exp(- static_cast<double>(time) / conf.TAU_M);
}

inline double Neuron::refractoryPotential(const long time) {
    return conf.DELTA_RP * exp(- static_cast<double>(time) / conf.TAU_RP);
}

inline double Neuron::adaptationPotentialDecay(const long time) {
    m_adaptation_potential *= exp(- static_cast<double>(time) / conf.TAU_SRA);
}

void Neuron::updateState(const long time) {
    auto dt = time - m_referenceTime;
    m_referenceTime = time;
    m_lifeSpan += dt;
    m_spikeRate = 1000000 * static_cast<double>(m_totalSpike) / static_cast<double>(m_lifeSpan);
}

inline void Neuron::thresholdAdaptation() { // TODO: weird use of time here
    m_recentSpikes.pop_back();
    m_recentSpikes.push_front(m_countSpike);
    resetSpikeCounter();

    for (size_t count : m_recentSpikes) {
        m_recentSpikeRate += static_cast<double>(count);
    }
    m_recentSpikeRate /= Conf::TIME_WINDOW_SR;

    if (m_recentSpikeRate > conf.TARGET_SPIKE_RATE) {
        m_threshold += conf.ETA_SR * (1 - exp(conf.TARGET_SPIKE_RATE - m_recentSpikeRate));
    } else {
        m_threshold -= conf.ETA_SR * (1 - exp(m_recentSpikeRate - conf.TARGET_SPIKE_RATE));
    }

    if (m_threshold < conf.MIN_THRESH) {
        m_threshold = conf.MIN_THRESH;
    }

    if (conf.TRACKING == "partial") {
        m_trackingThresholds.push_back(m_threshold);
    }
}

inline void Neuron::spikeRateAdaptation() {
    m_adaptation_potential += conf.DELTA_SRA;
}

inline bool Neuron::hasSpiked() {
    if (m_spike){
        m_spike = false;
        return true;
    }
    return false;
}

inline void Neuron::inhibition() {
    m_potential -= conf.DELTA_INH;
}

void Neuron::saveState(std::string &fileName) {
    json state;

    writeJson(state);

    std::ofstream ofs(fileName + ".json");
    if (ofs.is_open()) {
        ofs << std::setw(4) << state << std::endl;
    } else {
        std::cout << "cannot save neuron state file" << std::endl;
    }
    ofs.close();
}

void Neuron::loadState(std::string &fileName) {
    json state;
    std::ifstream ifs(fileName + ".json");
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

void Neuron::writeJson(json &state) {
    std::vector<size_t> position = {m_pos.posx(), m_pos.posy(), m_pos.posz()};
    std::vector<size_t> offset = {m_offset.posx(), m_offset.posy(), m_offset.posz()};
    std::vector<size_t> inIndex;
    std::vector<size_t> outIndex;
    std::vector<size_t> inhibIndex;
    for (auto neuron : m_inConnections) {
        inIndex.push_back(neuron.get().getIndex());
    }
    for (auto neuron : m_outConnections) {
        outIndex.push_back(neuron.get().getIndex());
    }
    for (auto neuron : m_inhibitionConnections) {
        inhibIndex.push_back(neuron.get().getIndex());
    }
    state["position"] = position;
    state["offset"] = offset;
    state["in_connections"] = inIndex;
    state["out_connections"] = outIndex;
    state["inhibition_connections"] = inhibIndex;
    state["potential"] = m_potential;
    state["count_spike"] = m_totalSpike;
    state["threshold"] = m_threshold;
    state["lifespan"] = m_lifeSpan;
    state["spiking_rate"] = m_spikeRate;
    state["recent_spikes"] = m_recentSpikes;
    state["learning_decay"] = m_learningDecay;
    state["thresholds_train"] = m_trackingThresholds;
    state["spike_train"] = m_trackingSpikeTrain;
    //    state["potential_train"] = m_trackingPotentialTrain;
}

void Neuron::readJson(const json &state) {
    m_totalSpike = state["count_spike"];
    m_threshold = state["threshold"];
    m_lifeSpan = state["lifespan"];
    m_recentSpikeRate = state["spiking_rate"];
    m_learningDecay = state["learning_decay"];
//    m_potential = state["potential"];
    m_recentSpikes.clear();
    for (size_t i = 0; i < Conf::TIME_WINDOW_SR; ++i) {
        m_recentSpikes.push_front(state["recent_spikes"][i]);
    }
}

void Neuron::trackPotential(const long time) {
    double potential = getPotential(time);
    m_trackingPotentialTrain.emplace_back(potential, time);
}
