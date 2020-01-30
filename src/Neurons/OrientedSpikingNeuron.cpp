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
    m_spikeCount = 0;
}

double OrientedSpikingNeuron::getPotential(const long time) {
    m_potential = potentialDecay(time - m_timestampLastEvent);
    return m_potential;
}

inline void OrientedSpikingNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    m_events.emplace_back(timestamp, x, y, polarity);
    update(timestamp, x, y, polarity);
}

/*inline void OrientedSpikingNeuron::newEventPot(const long timestamp, const int x, const int y, const bool polarity) {
    if (m_potential > -2 && m_potentials < 0) {
        m_weights(polarity, x, y) += exp(- m_potentials - 2) - 1;
    } else {
        m_weights(polarity, x, y) += exp(m_potentials - 2) - 1;
    }
}*/

inline bool OrientedSpikingNeuron::update(const long timestamp, const int x, const int y, const bool polarity) {
    long dt_event = timestamp - m_timestampLastEvent;
    m_potential = potentialDecay(dt_event);
    m_timestampLastEvent = timestamp;

    m_potential += m_weights(polarity, y, x);

    if (m_potential > m_threshold) {
        return spike();
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

    /***** Weights Normalization *****/
    ++m_countNormalize;
    if (m_countNormalize > NORMALIZATION_FREQUENCY) {
        normalize();
    }
}

inline bool OrientedSpikingNeuron::spike() {
    ++m_spikeCount;
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = m_events.back().timestamp();
    m_potential = VRESET;
    learnWeightsSTDP();
    return true;
}

inline void OrientedSpikingNeuron::normalize() {
    m_countNormalize = 0;
    for (int i = 0; i < 2; ++i) {
        double norm = xt::linalg::norm(xt::view(m_weights, i));
        if (norm != 0) {
            xt::view(m_weights, i) = NORMALIZATION_FACTOR * (xt::view(m_weights, i) / norm);
        }
    }
}

long OrientedSpikingNeuron::getSpikeFrequency(int timeInterval) {
    long freq = timeInterval * m_spikeCount / 1000000;
    m_spikeCount = 0;
    return freq;
}
