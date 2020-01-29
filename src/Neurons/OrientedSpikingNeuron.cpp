#include "OrientedSpikingNeuron.hpp"

OrientedSpikingNeuron::OrientedSpikingNeuron(int x, int y, xt::xarray<double> weights, double threshold) {
    m_events = std::vector<Event>();
    m_x = x;
    m_y = y;
    m_weights = std::move(weights);
    m_threshold = threshold;

    m_potential = 0;
    m_timestampLastEvent = 0;
    m_countNormalize = 0;
    m_spikingTime = 0;
    m_lastSpikingTime = 0;
}

double OrientedSpikingNeuron::getPotential(const long time) {
    m_potential = potentialDecay(time - m_timestampLastEvent);
    return m_potential;
}

inline void OrientedSpikingNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    m_events.emplace_back(timestamp, x, y, polarity);
    update(timestamp, x, y, polarity);
}

inline void OrientedSpikingNeuron::newEventPot(const long timestamp, const int x, const int y, const bool polarity) {
    
}

inline bool OrientedSpikingNeuron::update(const long timestamp, const int x, const int y, const bool polarity) {
    long dt_event = timestamp - m_timestampLastEvent;
    m_potential = potentialDecay(dt_event);
    m_timestampLastEvent = timestamp;

    m_potential += m_weights(polarity, y, x);

    if (m_potential > m_threshold) {
        return fire();
    }
    return false;
}

inline void OrientedSpikingNeuron::learnWeightsSTDP() {
    for (Event &event : m_events) {
        /***** Weights Potentiation *****/
        m_weights(event.polarity(), event.y(), event.x()) += DELTA_VP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / TAU_LTP);
        /***** Weights Depression *****/
        m_weights(event.polarity(), event.y(), event.x()) -= DELTA_VD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / TAU_LTD);
        if (m_weights(event.polarity(), event.y(), event.x()) < 0) {
            m_weights(event.polarity(), event.y(), event.x()) = 0;
        }
    }
    m_events.clear();

    ++m_countNormalize;
    if (m_countNormalize > NORMALIZATION_THRESHOLD) {
        m_countNormalize = 0;
        normalizeMatrix(m_weights);
    }
}

inline bool OrientedSpikingNeuron::fire() {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = m_events.back().timestamp();
    m_potential = VRESET;
    learnWeightsSTDP();
    return true;
}