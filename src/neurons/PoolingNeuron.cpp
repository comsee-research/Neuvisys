#include "PoolingNeuron.hpp"

PoolingNeuron::PoolingNeuron(NeuronConfig &conf, Luts &luts, int x, int y, xt::xarray<double> &weights) :
    Neuron(conf, luts, x, y, weights),
    m_events(std::vector<NeuronEvent>()) {
}

inline void PoolingNeuron::newEvent(const long timestamp, const int x, const int y, const int z) {
    membraneUpdate(timestamp, x, y, z);
    m_events.emplace_back(timestamp, x, y, z);
}

inline void PoolingNeuron::membraneUpdate(const long timestamp, const int x, const int y, const int z) {
    potentialDecay(timestamp - m_timestampLastEvent);
    m_potential += m_weights(z, y, x);
    m_timestampLastEvent = timestamp;

    if (m_potential > m_threshold) {
        spike(timestamp);
    }
}

inline void PoolingNeuron::spike(const long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_countSpike;
    ++m_totalSpike;
    m_potential = conf.VRESET;

//    spikeRateAdaptation();
    updateSTDP();
    m_events.clear();

    // Tracking
//    m_spikeTrain.push_back(time);
}

inline void PoolingNeuron::updateSTDP() {
    for (NeuronEvent &event : m_events) {
        if (static_cast<double>(m_spikingTime - event.timestamp()) < conf.TAU_LTP) {
            m_weights(event.z(), event.y(), event.x()) += conf.DELTA_VP;
        }

        if (m_weights(event.z(), event.y(), event.x()) < 0) {
            std::cout << "wtf" << std::endl;
            m_weights(event.z(), event.y(), event.x()) = 0;
        }
    }

    normalizeWeights();
}

inline void PoolingNeuron::normalizeWeights() {
    double norm = 0;
    for (auto val : xt::ravel<xt::layout_type::row_major>(m_weights)) {
        norm += val * val;
    }
    norm = sqrt(norm);
    if (norm != 0) {
        m_weights = conf.NORM_FACTOR * (m_weights / norm);
    }
}

double PoolingNeuron::getWeights(int x, int y, int z) {
    return m_weights(z, y, x);
}
