#include "OrientedNeuron.hpp"

#include <utility>

OrientedNeuron::OrientedNeuron(int x, int y, xt::xarray<double> weights, double threshold) : Neuron(x, y, std::move(weights), threshold) {
    m_events = std::vector<Event>();
    m_spikingTime = 0;
    m_lastSpikingTime = 0;
}

inline double OrientedNeuron::getPotential(const long time) {
    return m_potential * potentialDecay(time - m_timestampLastEvent);
}

inline void OrientedNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    m_events.emplace_back(timestamp, x, y, polarity);
    internalUpdate(timestamp, x, y, polarity);
}

inline bool OrientedNeuron::internalUpdate(const long timestamp, const int x, const int y, const bool polarity) {
    double decay = m_potential * potentialDecay(timestamp - m_timestampLastEvent);
    double grp = 1.2 * refractoryPeriod(timestamp - m_spikingTime);
    m_potential = decay + m_weights(polarity, y, x) - grp;

    m_timestampLastEvent = timestamp;

    if (m_potential > m_threshold) {
        spike(timestamp);
    }
}

inline void OrientedNeuron::spike(long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_countSpike;
    ++m_totalSpike;
    m_potential = VRESET;
    learnWeightsSTDP();
    m_events.clear();

    // Tracking
    m_spikeTrain.push_back(time);
}

inline void OrientedNeuron::learnWeightsSTDP() {
    for (Event &event : m_events) {
        /***** Weights Potentiation *****/
        m_weights(event.polarity(), event.y(), event.x()) += DELTA_VP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / TAU_LTP);
        /***** Weights Depression *****/
        m_weights(event.polarity(), event.y(), event.x()) -= DELTA_VD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / TAU_LTD);
        if (m_weights(event.polarity(), event.y(), event.x()) < 0) {
            m_weights(event.polarity(), event.y(), event.x()) = 0;
        }
    }

    /***** Weights Normalization *****/
    if (m_totalSpike % NORM_THRESHOLD == 0) {
        normalize();
    }
}

inline void OrientedNeuron::normalize() {
    for (int i = 0; i < 2; ++i) {
        double norm = xt::linalg::norm(xt::view(m_weights, i));
        if (norm != 0) {
            xt::view(m_weights, i) = NORM_FACTOR * (xt::view(m_weights, i) / norm);
        }
    }
}
