#include "Neuron.hpp"

using json = nlohmann::json;

Neuron::Neuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset) :
        m_index(index),
        m_layer(layer),
        conf(conf),
        m_pos(pos),
        m_offset(offset),
        m_trackingSpikeTrain(std::vector<long>(0)) {
    m_threshold = conf.VTHRESH;
    m_learningDecay = 1.0;
    m_spike = false;
}

inline double Neuron::getPotential(long time) {
    return m_potential * exp(- static_cast<double>(time - m_timestampLastEvent) / conf.TAU_M);
}

inline double Neuron::potentialDecay(long time) {
    m_potential *= exp(- static_cast<double>(time) / conf.TAU_M);
}

inline double Neuron::refractoryPotential(long time) {
    return conf.DELTA_RP * exp(- static_cast<double>(time) / conf.TAU_RP);
}

inline double Neuron::adaptationPotentialDecay(long time) {
    m_adaptationPotential *= exp(- static_cast<double>(time) / conf.TAU_SRA);
}

void Neuron::updateState(long timeInterval, double alpha) {
    m_lifeSpan += timeInterval;
    double spikesPerSecond = static_cast<double>(m_countSpike) * (E6 / static_cast<double>(timeInterval)); // spikes/s
    resetSpikeCounter();
    m_spikingRateAverage = (alpha * spikesPerSecond) + (1.0 - alpha) * m_spikingRateAverage; // exponential rolling average
}

inline void Neuron::thresholdAdaptation() {
    if (m_spikingRateAverage > conf.TARGET_SPIKE_RATE) {
        m_threshold += conf.ETA_SR * (1 - exp(conf.TARGET_SPIKE_RATE - m_spikingRateAverage));
    } else {
        m_threshold -= conf.ETA_SR * (1 - exp(m_spikingRateAverage - conf.TARGET_SPIKE_RATE));
    }

    if (m_threshold < conf.MIN_THRESH) {
        m_threshold = conf.MIN_THRESH;
    }
}

inline void Neuron::spikeRateAdaptation() {
    m_adaptationPotential += conf.DELTA_SRA;
}

inline bool Neuron::hasSpiked() {
    if (m_spike){
        m_spike = false;
        return true;
    }
    return false;
}

inline void Neuron::inhibition() {
    m_potential -= conf.ETA_INH;
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
    std::vector<size_t> position = {m_pos.x(), m_pos.y(), m_pos.z()};
    std::vector<size_t> offset = {m_offset.x(), m_offset.y(), m_offset.z()};
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
    state["speed"] = position;
    state["offset"] = offset;
    state["in_connections"] = inIndex;
    state["out_connections"] = outIndex;
    state["inhibition_connections"] = inhibIndex;
    state["potential"] = m_potential;
    state["count_spike"] = m_totalSpike;
    state["threshold"] = m_threshold;
    state["lifespan"] = m_lifeSpan;
    state["spiking_rate"] = m_spikingRateAverage;
    state["learning_decay"] = m_learningDecay;
    state["spike_train"] = m_trackingSpikeTrain;
    //    state["potential_train"] = m_trackingPotentialTrain;
}

void Neuron::readJson(const json &state) {
    m_totalSpike = state["count_spike"];
    m_threshold = state["threshold"];
    m_lifeSpan = state["lifespan"];
    m_learningDecay = state["learning_decay"];
    m_spikingRateAverage = state["spiking_rate"];
//    m_potential = state["potential"];
}

void Neuron::trackPotential(const long time) {
    double potential = getPotential(time);
    m_trackingPotentialTrain.emplace_back(potential, time);
}
