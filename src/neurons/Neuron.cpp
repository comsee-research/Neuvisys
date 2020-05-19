#include "Neuron.hpp"
#include <cmath>

using json = nlohmann::json;

Neuron::Neuron(int x, int y, xt::xarray<double> weights, double threshold) {
    m_x = x;
    m_y = y;
    m_weights = std::move(weights);
    m_threshold = threshold;

    m_recentSpikes = std::list<int>(TIME_WINDOW_SR);
    m_totalSpike = 0;
    m_countSpike = 0;
    m_potential = 0;
    m_spike = false;
    m_spikingRate = 0;
    m_timestampLastEvent = 0;

    // Tracking Variables
    m_spikeTrain = std::vector<long>(0);
}

inline int Neuron::getX() {
    return m_x;
}

inline int Neuron::getY() {
    return m_y;
}

inline double Neuron::getWeights(const int p, const int x, const int y) {
    return m_weights(p, y, x);
}

inline double Neuron::getThreshold() {
    return m_threshold;
}

inline double Neuron::getPotential(long time) {
    return 0;
}

double Neuron::getSpikingRate() {
    return m_spikingRate;
}

inline double Neuron::potentialDecay(const long time) {
    return exp(- static_cast<double>(time) / TAU_M);
}

inline double Neuron::refractoryPeriod(const long time) {
    return exp(- static_cast<double>(time) / TAU_RP);
}

inline void Neuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {

}

inline void Neuron::thresholdAdaptation() {
    m_recentSpikes.pop_back();
    m_recentSpikes.push_front(m_countSpike);
    m_countSpike = 0;

    for (int count : m_recentSpikes) {
        m_spikingRate += count;
    }
    m_spikingRate /= TIME_WINDOW_SR;

    m_threshold += DELTA_SR * (m_spikingRate - TARGET_SPIKE_RATE);

    if (m_threshold < 20) {
        m_threshold = 20;
    }
}

inline void Neuron::spike() {
    m_potential = VRESET;
    m_spike = true;
}

inline bool Neuron::hasSpiked() {
    if (m_spike){
        m_spike = false;
        return true;
    }
    return false;
}

inline void Neuron::inhibition() {
    m_potential -= DELTA_INH;
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
        m_recentSpikes.clear();
        for (size_t i = 0; i < TIME_WINDOW_SR; ++i) {
            m_recentSpikes.push_front(conf["recent_spikes"][i]);
        }
/*        for (size_t i = 0; i < conf["spike_train"].size(); ++i) {
            m_spikeTrain.push_back(conf["m_spikeTrain"][i]);
        }*/
    } else {
        std::cout << "cannot open neuron state file" << std::endl;
    }
    ifs.close();
}