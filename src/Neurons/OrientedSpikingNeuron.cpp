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

void OrientedSpikingNeuron::newEventPot(const long timestamp, const int x, const int y, const bool polarity) {
    if (m_potential > 5) {
        m_weights(polarity, x, y) += 0.016;
    } else if (m_potential > -5 && m_weights(polarity, x, y) > 0.008){
        m_weights(polarity, x, y) -= 0.008;
    }
/*    if (m_weights(polarity, y, x) < 0.) {
        m_weights(polarity, y, x) = 0.;
    }*/
    update(timestamp, x, y, polarity);
}

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
    if (m_countNormalize > NORM_THRESHOLD) {
        normalize();
    }
}

inline bool OrientedSpikingNeuron::spike() {
    ++m_spikeCount;
//    m_lastSpikingTime = m_spikingTime;
//    m_spikingTime = m_events.back().timestamp();
    m_potential = VRESET;
//    learnWeightsSTDP();
    return true;
}

void OrientedSpikingNeuron::normalize() {
    m_countNormalize = 0;
    for (int i = 0; i < 2; ++i) {
        double norm = xt::linalg::norm(xt::view(m_weights, i));
        if (norm != 0) {
            xt::view(m_weights, i) = NORM_FACTOR * (xt::view(m_weights, i) / norm);
        }
    }
}

inline void OrientedSpikingNeuron::adaptThreshold() {
    m_threshold += 0.1 * (m_spikeCount - 20);
}

void OrientedSpikingNeuron::resetSpikeCount() {
    m_spikeCount = 0;
}

int OrientedSpikingNeuron::getSpikeCount() {
    return m_spikeCount;
}