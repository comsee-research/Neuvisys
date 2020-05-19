#include "TemporalNeuron.hpp"

TemporalNeuron::TemporalNeuron(int x, int y, xt::xarray<double> weights, xt::xarray<long> delays, double threshold) : Neuron(x, y, std::move(weights), threshold){
    m_delays = std::move(delays);
}

long TemporalNeuron::getDelay(const int x, const int y) {
    return m_delays(y, x);
}

long TemporalNeuron::getTimestampNextEvent() {
    return m_events.top().timestamp();
}

double TemporalNeuron::getPotential(const long time) {
    return potentialDecay(time - m_timestampLastEvent);
}

inline void TemporalNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    m_events.emplace(timestamp + m_delays(y, x), x, y, polarity);
}

inline bool TemporalNeuron::update(long time) {
    while (!m_events.empty() && getTimestampNextEvent() <= time) {
        Event event = m_events.top();
        m_events.pop();

        long dt_event = event.timestamp() - m_timestampLastEvent;
        m_potential = potentialDecay(dt_event);
        m_timestampLastEvent = event.timestamp();

        m_potential += m_weights(event.polarity(), event.y(), event.x());

        if (m_potential > m_threshold) {
            spike();
            return true;
        }
    }
    return false;
}

void TemporalNeuron::spike() {
    m_events = std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>();
    m_potential = 0;
    m_spike = true;
}
