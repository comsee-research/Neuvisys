#include "SpatioTemporalNeuron.hpp"

SpatioTemporalNeuron::SpatioTemporalNeuron(NeuronConfig &conf, Luts &luts, int x, int y, xt::xarray<double> &weights, int nbSynapses) :
    Neuron(conf, luts, x, y, weights),
    m_events(std::vector<Event>()),
    m_waitingList(std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>()) {
    for (int synapse = 0; synapse < nbSynapses; synapse++) {
        m_delays.push_back(synapse * conf.SYNAPSE_DELAY);
    }
}

inline void SpatioTemporalNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    if ((m_delays.size() == 1) && (m_delays[0] == 0)) {
        membraneUpdate(timestamp, x, y, polarity, 0);
        m_events.emplace_back(timestamp, x, y, polarity, 0);
    } else {
        int synapse = 0;
        for (auto delay : m_delays) {
            m_waitingList.emplace(timestamp + delay, x, y, polarity, synapse++);
        }
    }
}

void SpatioTemporalNeuron::update(const long time) {
    while (!m_waitingList.empty() && m_waitingList.top().timestamp() <= time) {
        Event event = m_waitingList.top();
        m_waitingList.pop();
        m_events.push_back(event);

        membraneUpdate(event.timestamp(), event.x(), event.y(), event.polarity(), event.synapse());
    }
}

inline void SpatioTemporalNeuron::membraneUpdate(const long timestamp, const int x, const int y, const bool polarity, const int synapse) {
    potentialDecay(timestamp - m_timestampLastEvent);
    adaptationPotentialDecay(timestamp - m_timestampLastEvent);
    m_potential += m_weights(polarity, synapse, y, x)
                   - refractoryPotential(timestamp - m_spikingTime)
                   - m_adaptation_potential;
    m_timestampLastEvent = timestamp;

    if (m_potential > m_threshold) {
        spike(timestamp);
    }
}

inline void SpatioTemporalNeuron::spike(const long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_countSpike;
    ++m_totalSpike;
    m_potential = conf.VRESET;

    spikeRateAdaptation();
    updateSTDP();
    m_events.clear();

    // Tracking
//    m_spikeTrain.push_back(time);
}

inline void SpatioTemporalNeuron::updateSTDP() {
    for (Event &event : m_events) {
        m_weights(event.polarity(), event.synapse(), event.y(), event.x()) += m_learningDecay * conf.DELTA_VP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / conf.TAU_LTP);
        m_weights(event.polarity(), event.synapse(), event.y(), event.x()) -= m_learningDecay * conf.DELTA_VD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / conf.TAU_LTD);

        if (m_weights(event.polarity(), event.synapse(), event.y(), event.x()) < 0) {
            m_weights(event.polarity(), event.synapse(), event.y(), event.x()) = 0;
        }
    }

    normalizeWeights();
    m_learningDecay *= conf.DECAY_FACTOR;
}

inline void SpatioTemporalNeuron::normalizeWeights() {
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < m_delays.size(); ++j) {
            double norm = xt::linalg::norm(xt::view(m_weights, i, j), 1);
            if (norm != 0) {
                xt::view(m_weights, i, j) = conf.NORM_FACTOR * (xt::view(m_weights, i, j) / norm);
            }
        }
    }
}

double SpatioTemporalNeuron::getWeights(int p, int s, int x, int y) {
    return m_weights(p, s, y, x);
}
