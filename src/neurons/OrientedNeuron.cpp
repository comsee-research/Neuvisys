#include "OrientedNeuron.hpp"

#include <utility>

OrientedNeuron::OrientedNeuron(int x, int y, xt::xarray<double> weights, double threshold) : Neuron(x, y, std::move(weights), threshold) {
    m_events = std::vector<Event>();
    m_countNormalize = 0;
    m_spikingTime = 0;
    m_lastSpikingTime = 0;
}

inline double OrientedNeuron::getPotential(const long time) {
    return potentialDecay(time - m_timestampLastEvent);
}

inline bool OrientedNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    if (timestamp > m_inhibitionTime + INHIBITION) {
        m_events.emplace_back(timestamp, x, y, polarity);
        return update(timestamp, x, y, polarity);
    }
    return false;
}

inline bool OrientedNeuron::newEventPot(const long timestamp, const int x, const int y, const bool polarity) {
    if (m_potential > 5) {
        m_weights(polarity, x, y) += 0.016;
    } else if (m_potential > -5 && m_weights(polarity, x, y) > 0.008){
        m_weights(polarity, x, y) -= 0.008;
    }
    update(timestamp, x, y, polarity);
}

inline bool OrientedNeuron::update(const long timestamp, const int x, const int y, const bool polarity) {
    long dt_event = timestamp - m_timestampLastEvent;
    m_potential = potentialDecay(dt_event);
    m_timestampLastEvent = timestamp;

    m_potential += m_weights(polarity, y, x);

    if (m_potential > m_threshold) {
        spike(timestamp);
        return true;
    }
    return false;
}

inline void OrientedNeuron::spike(long time) {
    m_spikingTime = time;
    m_lastSpikingTime = m_spikingTime;
    learnWeightsSTDP();
    m_potential = VRESET;
    m_spike = true;
    m_events.clear();
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
    ++m_countNormalize;
    if (m_countNormalize > NORM_THRESHOLD) {
        normalize();
    }
}

inline void OrientedNeuron::normalize() {
    m_countNormalize = 0;
    for (int i = 0; i < 2; ++i) {
        double norm = xt::linalg::norm(xt::view(m_weights, i));
        if (norm != 0) {
            xt::view(m_weights, i) = NORM_FACTOR * (xt::view(m_weights, i) / norm);
        }
    }
}
