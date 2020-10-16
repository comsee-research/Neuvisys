#include "SimpleNeuron.hpp"

SimpleNeuron::SimpleNeuron(size_t index, NeuronConfig &conf, Luts &luts, Position pos, Position offset, xt::xarray<double> &weights, size_t nbSynapses) :
    Neuron(index,conf, luts, pos, offset, weights),
    m_events(boost::circular_buffer<Event>(1000)),
    m_waitingList(std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>()) {
    for (size_t synapse = 0; synapse < nbSynapses; synapse++) {
        m_delays.push_back(static_cast<long>(synapse * conf.SYNAPSE_DELAY));
    }
}

inline void SimpleNeuron::newEvent(const long timestamp, const size_t x, const size_t y, const bool polarity) {
    if ((m_delays.size() == 1) && (m_delays[0] == 0)) {
        membraneUpdate(timestamp, x, y, polarity, 0);
        m_events.push_back(Event(timestamp, x, y, polarity, 0));
    } else {
        size_t synapse = 0;
        for (auto delay : m_delays) {
            m_waitingList.emplace(timestamp + delay, x, y, polarity, synapse++);
        }
    }
}

void SimpleNeuron::update(const long time) {
    while (!m_waitingList.empty() && m_waitingList.top().timestamp() <= time) {
        Event event = m_waitingList.top();
        m_waitingList.pop();
        m_events.push_back(event);

        membraneUpdate(event.timestamp(), event.x(), event.y(), event.polarity(), event.synapse());
    }
}

inline void SimpleNeuron::membraneUpdate(const long timestamp, const size_t x, const size_t y, const bool polarity, const size_t synapse) {
    potentialDecay(timestamp - m_timestampLastEvent);
    adaptationPotentialDecay(timestamp - m_timestampLastEvent);
    m_potential += m_weights(polarity, synapse, x, y)
                   - refractoryPotential(timestamp - m_spikingTime)
                   - m_adaptation_potential;
    m_timestampLastEvent = timestamp;

    if (m_potential > m_threshold) {
        spike(timestamp);
    }
}

inline void SimpleNeuron::spike(const long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_countSpike;
    ++m_totalSpike;
    m_potential = conf.VRESET;

    spikeRateAdaptation();
    if (conf.STDP_LEARNING) {
        updateSTDP();
    }
    m_events.clear();

    if (conf.TRACKING) {
        m_spikeTrain.push_back(time);
    }
}

inline void SimpleNeuron::updateSTDP() {
    for (Event &event : m_events) {
        m_weights(event.polarity(), event.synapse(), event.x(), event.y()) += m_learningDecay * conf.DELTA_VP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / conf.TAU_LTP);
        m_weights(event.polarity(), event.synapse(), event.x(), event.y()) -= m_learningDecay * conf.DELTA_VD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / conf.TAU_LTD);

        if (m_weights(event.polarity(), event.synapse(), event.x(), event.y()) < 0) {
            m_weights(event.polarity(), event.synapse(), event.x(), event.y()) = 0;
        }
    }

    normalizeWeights();
//    m_learningDecay = 1 / (1 + exp(m_totalSpike - conf.DECAY_FACTOR));
}

inline void SimpleNeuron::normalizeWeights() {
    for (size_t p = 0; p < 2; ++p) {
        for (size_t s = 0; s < m_delays.size(); ++s) {
            double norm = xt::linalg::norm(xt::view(m_weights, p, s));
            if (norm != 0) {
                xt::view(m_weights, p, s) = conf.NORM_FACTOR * (xt::view(m_weights, p, s) / norm);
            }
        }
    }
}

double SimpleNeuron::getWeights(size_t p, size_t s, size_t x, size_t y) {
    return m_weights(p, s, x, y);
}
