#include "Neuron.hpp"
#include <cmath>

using json = nlohmann::json;

Neuron::Neuron(NeuronConfig &conf, Luts &luts, int x, int y, xt::xarray<double> &weights) :
    conf(conf),
    m_weights(weights),
    m_recentSpikes(std::list<int>(Conf::TIME_WINDOW_SR)),
    m_luts(luts),
    m_spikeTrain(std::vector<long>(0)) {
    m_x = x;
    m_y = y;
    m_threshold = conf.VTHRESH;
    m_learningDecay = 1.0;
    m_spike = false;
    m_connections = xt::zeros<size_t>(m_weights.shape());
}

inline double Neuron::getPotential(const long time) {
    return m_potential * exp(- static_cast<double>(time - m_timestampLastEvent) / conf.TAU_M);
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

    for (int count : m_recentSpikes) {
        m_spikingRate += count;
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
    xt::dump_npy(fileName + ".npy", m_weights);
    xt::dump_npy(fileName + "_conn.npy", m_connections);
    json state;

    state["potential"] = m_potential;
    state["count_spike"] = m_totalSpike;
    state["threshold"] = m_threshold;
    state["creation_time"] = m_creationTime;
    state["spiking_rate"] = m_spikingRate;
    state["recent_spikes"] = m_recentSpikes;
    state["spike_train"] = m_spikeTrain;
    state["learning_decay"] = m_learningDecay;

    std::ofstream ofs(fileName + ".json");
    if (ofs.is_open()) {
        ofs << std::setw(4) << state << std::endl;
    } else {
        std::cout << "cannot save neuron state file" << std::endl;
    }
    ofs.close();
}

void Neuron::loadState(std::string &fileName) {
    try {
        m_weights = xt::load_npy<double>(fileName + ".npy");
    } catch (const std::exception &exe) {
//        std::cout << "No starting weights, random initialization" << std::endl;
    }

    json state;
    std::ifstream ifs(fileName + ".json");
    if (ifs.is_open()) {
        try {
            ifs >> state;
        } catch (const std::exception& e) {
            std::cerr << "In Neuron state file" << std::endl;
            throw;
        }
//        m_potential = state["potential"];
        m_totalSpike = state["count_spike"];
        m_threshold = state["threshold"];
        m_creationTime = state["creation_time"];
        m_spikingRate = state["spiking_rate"];
        m_learningDecay = state["learning_decay"];
        m_recentSpikes.clear();
        for (size_t i = 0; i < Conf::TIME_WINDOW_SR; ++i) {
            m_recentSpikes.push_front(state["recent_spikes"][i]);
        }
        for (auto &spikes : state["spike_train"]) {
            m_spikeTrain.push_back(spikes);
        }
    } else {
//        std::cout << "cannot open neuron state file" << std::endl;
    }
    ifs.close();
}