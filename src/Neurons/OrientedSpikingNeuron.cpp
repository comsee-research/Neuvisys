#include "OrientedSpikingNeuron.hpp"

OrientedSpikingNeuron::OrientedSpikingNeuron(int x, int y, xt::xarray<double> weightsOn, xt::xarray<double> weightsOff, double threshold) {
    m_events = std::vector<Event>();
    m_x = x;
    m_y = y;
    m_weightsOn = std::move(weightsOn);
    m_weightsOff = std::move(weightsOff);
    m_threshold = threshold;
    m_potential = 0.f;
    m_timestampLastEvent = 0;
    m_countNormalize = 0;
}

double OrientedSpikingNeuron::getPotential(const long time) {
    m_potential = potentialDecay(time - m_timestampLastEvent);
    return m_potential;
}

inline void OrientedSpikingNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    m_events.emplace_back(timestamp, x, y, polarity);
    update(timestamp, x, y, polarity);
}

inline bool OrientedSpikingNeuron::update(const long timestamp, const int x, const int y, const bool polarity) {
    long dt_event = timestamp - m_timestampLastEvent;
    m_potential = potentialDecay(dt_event);
    m_timestampLastEvent = timestamp;

    if (polarity) {
        m_potential += m_weightsOn(y, x);
    } else {
        m_potential += m_weightsOff(y, x);
    }

    if (m_potential > m_threshold) {
        return fire();
    }
    return false;
}

inline void OrientedSpikingNeuron::learnWeightsSTDP() {
    long actualTime = m_events.back().timestamp();
    for (Event &event : m_events) {
        if (event.polarity()) {
            m_weightsOn(event.y(), event.x()) *= 1 + 0.1 * exp(- static_cast<double>(actualTime - event.timestamp()) / DECAY);
        } else {
            m_weightsOff(event.y(), event.x()) *= 1 + 0.1 * exp(- static_cast<double>(actualTime - event.timestamp()) / DECAY);
        }
    }
    m_events.clear();

    ++m_countNormalize;
    if (m_countNormalize > NORMALIZATION_THRESHOLD) {
        m_countNormalize = 0;
        normalizeMatrix(m_weightsOn);
        normalizeMatrix(m_weightsOff);
    }
}

inline bool OrientedSpikingNeuron::fire() {
    learnWeightsSTDP();
    m_potential = VRESET;
    return true;
}