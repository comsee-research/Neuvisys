#include "Neuron.hpp"
#include <cmath>

using json = nlohmann::json;

Neuron::Neuron(NeuronConfig &conf, Luts &luts, int x, int y, xt::xarray<double> &weights) : conf(conf), m_weights(weights), m_luts(luts) {
    m_x = x;
    m_y = y;
    m_threshold = conf.VTHRESH;
    m_learningDecay = 1.0;
    m_recentSpikes = std::list<int>(Conf::TIME_WINDOW_SR);
    m_spike = false;

    // Tracking Variables
    m_spikeTrain = std::vector<long>(0);
}

inline double Neuron::getPotential(const long time) {
    return m_potential * exp(- static_cast<double>(time - m_timestampLastEvent) / conf.TAU_M);
}

inline double Neuron::potentialDecay(const long time) {
//    m_potential *= exp(- static_cast<double>(time) / conf.TAU_M);
    if (time < 1000000) {
        m_potential *= m_luts.lutM[time];
    } else {
        m_potential = 0;
    }
}

inline double Neuron::refractoryPotential(const long time) {
//    return conf.DELTA_RP * exp(- static_cast<double>(time) / conf.TAU_RP);
    if (time < 1000000) {
        return conf.DELTA_RP * m_luts.lutRP[time];
    } else {
        return 0;
    }
}

inline double Neuron::adaptationPotentialDecay(const long time) {
//    m_adaptation_potential *= exp(- static_cast<double>(time) / conf.TAU_SRA);
    if (time < 1000000) {
        m_adaptation_potential *= m_luts.lutM[time];
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

    if (m_threshold < 5) {
        m_threshold = 5;
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

    json conf;
    conf["potential"] = m_potential;
    conf["count_spike"] = m_totalSpike;
    conf["threshold"] = m_threshold;
    conf["creation_time"] = m_creationTime;
    conf["spiking_rate"] = m_spikingRate;
    conf["recent_spikes"] = m_recentSpikes;
    conf["spike_train"] = m_spikeTrain;
    conf["learning_decay"] = m_learningDecay;

    std::ofstream ofs(fileName + ".json");
    if (ofs.is_open()) {
        ofs << std::setw(4) << conf << std::endl;
    } else {
        std::cout << "cannot save neuron state file" << std::endl;
    }
    ofs.close();
}

void Neuron::loadState(std::string &fileName) {
    try {
        m_weights = xt::load_npy<double>(fileName + ".npy");
    } catch (std::exception exe) {
        std::cout << "No starting weights, random initialization" << std::endl;
    }

    std::ifstream ifs(fileName + ".json");
    if (ifs.is_open()) {
        json conf;
        ifs >> conf;

        m_potential = conf["potential"];
        m_totalSpike = conf["count_spike"];
        m_threshold = conf["threshold"];
        m_creationTime = conf["creation_time"];
        m_spikingRate = conf["spiking_rate"];
        m_learningDecay = conf["learning_decay"];

        m_recentSpikes.clear();
        for (size_t i = 0; i < Conf::TIME_WINDOW_SR; ++i) {
            m_recentSpikes.push_front(conf["recent_spikes"][i]);
        }
        for (auto &spikes : conf["spike_train"]) {
            m_spikeTrain.push_back(spikes);
        }
    } else {
        std::cout << "cannot open neuron state file" << std::endl;
    }
    ifs.close();
}