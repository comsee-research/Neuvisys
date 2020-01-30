#include "DelayedSpikingNeuron.hpp"

DelayedSpikingNeuron::DelayedSpikingNeuron(int x, int y, xt::xarray<double> weights, xt::xarray<long> delays, double threshold) {
    m_x = x;
    m_y = y;
    m_weights = std::move(weights);
    m_delays = std::move(delays);
    m_threshold = threshold;
    m_potential = 0.f;
    m_timestampLastEvent = 0;
}

long DelayedSpikingNeuron::getDelay(const int x, const int y) {
    return m_delays(y, x);
}

long DelayedSpikingNeuron::getTimestampNextEvent() {
    return m_events.top().timestamp();
}

double DelayedSpikingNeuron::getPotential(const long time) {
    return potentialDecay(time - m_timestampLastEvent);
}

void DelayedSpikingNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    m_events.emplace(timestamp + m_delays(y, x), x, y, polarity);
}

bool DelayedSpikingNeuron::update(long time) {
    while (!m_events.empty() && getTimestampNextEvent() <= time) {
        Event event = m_events.top();
        m_events.pop();

        long dt_event = event.timestamp() - m_timestampLastEvent;
        m_potential = potentialDecay(dt_event);
        m_timestampLastEvent = event.timestamp();

        m_potential += m_weights(event.polarity(), event.y(), event.x());

        if (m_potential > m_threshold) {
            return spike();
        }
    }
    return false;
}

bool DelayedSpikingNeuron::spike() {
    m_events = std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>();
    m_potential = 0;
    return true;
}
