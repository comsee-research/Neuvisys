#include "Neuron.hpp"

using json = nlohmann::json;

Neuron::Neuron(size_t index, NeuronConfig &conf, Luts &luts, Position pos, Position offset) :
    m_index(index),
    conf(conf),
    m_pos(pos),
    m_offset(offset),
    m_recentSpikes(std::list<size_t>(Conf::TIME_WINDOW_SR)),
    m_luts(luts),
    m_spikeTrain(std::vector<long>(0)) {
    m_threshold = conf.VTHRESH;
    m_learningDecay = 1.0;
    m_spike = false;
}

inline double Neuron::getPotential(const long time) {
    return m_potential * exp(- static_cast<double>(time - m_timestampLastEvent) / conf.TAU_M); // TODO (use LUT)
}

inline double Neuron::potentialDecay(const long time) {
//    m_potential *= exp(- static_cast<double>(time) / conf.TAU_M);
    if (time < 1000000) {
        m_potential *= m_luts.lutM[static_cast<size_t>(time)];
    } else {
        m_potential = 0;
    }
}

inline double Neuron::refractoryPotential(const long time) {
//    return conf.DELTA_RP * exp(- static_cast<double>(time) / conf.TAU_RP);
    if (time < 1000000) {
        return conf.DELTA_RP * m_luts.lutRP[static_cast<size_t>(time)];
    } else {
        return 0;
    }
}

inline double Neuron::adaptationPotentialDecay(const long time) {
//    m_adaptation_potential *= exp(- static_cast<double>(time) / conf.TAU_SRA);
    if (time < 1000000) {
        m_adaptation_potential *= m_luts.lutM[static_cast<size_t>(time)];
    } else {
        m_adaptation_potential = 0;
    }
}

inline void Neuron::thresholdAdaptation() {
    m_recentSpikes.pop_back();
    m_recentSpikes.push_front(m_countSpike);
    m_countSpike = 0;

    for (size_t count : m_recentSpikes) {
        m_spikingRate += static_cast<double>(count);
    }
    m_spikingRate /= Conf::TIME_WINDOW_SR;

    if (m_spikingRate > conf.TARGET_SPIKE_RATE) {
        m_threshold += conf.DELTA_SR * (1 - exp(conf.TARGET_SPIKE_RATE - m_spikingRate));
    } else {
        m_threshold -= conf.DELTA_SR * (1 - exp(m_spikingRate - conf.TARGET_SPIKE_RATE));
    }

    if (m_threshold < conf.MIN_THRESH) {
        m_threshold = conf.MIN_THRESH;
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
    state["creation_time"] = m_creationTime;
    state["spiking_rate"] = m_spikingRate;
    state["recent_spikes"] = m_recentSpikes;
    state["learning_decay"] = m_learningDecay;

    state["spike_train"] = m_spikeTrain;
    state["potential_train"] = m_potentialTrain;

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
        m_totalSpike = state["count_spike"];
        m_threshold = state["threshold"];
        m_creationTime = state["creation_time"];
        m_spikingRate = state["spiking_rate"];
        m_learningDecay = state["learning_decay"];
//        m_potential = state["potential"];
        m_recentSpikes.clear();
        for (size_t i = 0; i < Conf::TIME_WINDOW_SR; ++i) {
            m_recentSpikes.push_front(state["recent_spikes"][i]);
        }
//        for (auto &spikes : state["spike_train"]) {
//            m_spikeTrain.push_back(spikes);
//        }
    } else {
//        std::cout << "cannot open neuron state file" << std::endl;
    }
    ifs.close();
}

void Neuron::trackPotential(const long time) {
    if (conf.TRACKING) {
        double potential = getPotential(time);
        m_potentialTrain.emplace_back(potential, time);
    }
}