#include "PoolingNeuron.hpp"

PoolingNeuron::PoolingNeuron(NeuronConfig &conf, Luts &luts, int x, int y, xt::xarray<double> &weights) : Neuron(conf, luts, x, y, weights) {
    m_events = std::vector<NeuronEvent>();
}

inline void PoolingNeuron::newEvent(const long timestamp, const int x, const int y, const int z) {
    membraneUpdate(timestamp, x, y, z);
    m_events.emplace_back(timestamp, x, y, z);
}

inline void PoolingNeuron::membraneUpdate(const long timestamp, const int x, const int y, const int z) {
    potentialDecay(timestamp - m_timestampLastEvent);
//    adaptationPotentialDecay(timestamp - m_timestampLastEvent);
    m_potential += m_weights(z, y, x);
//                   - refractoryPotential(timestamp - m_spikingTime)
//                   - m_adaptation_potential;
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
    m_spikeTrain.push_back(time);
}

inline void PoolingNeuron::updateSTDP() {
    for (NeuronEvent &event : m_events) {
        if (m_spikingTime - event.timestamp() < 8000) { //TODO
            m_weights(event.z(), event.y(), event.x()) += conf.DELTA_VP;
        }
    }

    normalizeWeights();
}

inline void PoolingNeuron::normalizeWeights() {
    for (int layer = 0; layer < m_weights.shape()[0]; ++layer) {
        double norm = xt::linalg::norm(xt::view(m_weights, layer), 1);
        if (norm != 0) {
            xt::view(m_weights, layer) = conf.NORM_FACTOR * (xt::view(m_weights, layer) / norm);
        }
    }
}

double PoolingNeuron::getWeights(int x, int y, int z) {
    return m_weights(z, y, x);
}
