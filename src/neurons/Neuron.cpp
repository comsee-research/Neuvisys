#include "Neuron.hpp"
#include <cmath>

using json = nlohmann::json;

Neuron::Neuron(int x, int y, xt::xarray<double> weights, double threshold) {
    m_x = x;
    m_y = y;
    m_weights = std::move(weights);
    m_threshold = threshold;

    m_potential = 0;
    m_spike = false;
    m_countSpike = 0;
    m_timestampLastEvent = 0;
    m_inhibitionTime = 0;
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

int Neuron::getCountSpike() {
    return m_countSpike;
}

inline double Neuron::potentialDecay(const long time) {
    return m_potential * exp(- static_cast<double>(time) / TAU_M);
}

inline bool Neuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {

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

inline void Neuron::setInhibitionTime(long inhibitionTime) {
    m_inhibitionTime = inhibitionTime;
}

void Neuron::saveState(std::string &fileName) {
    xt::dump_npy(fileName + ".npy", m_weights);

    json conf;

    conf["potential"] = m_potential;
    conf["count_spike"] = m_countSpike;

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
        m_countSpike = conf["count_spike"];
    } else {
        std::cout << "cannot open neuron state file" << std::endl;
    }
    ifs.close();
}