#include "DelayedSpikingNeuron2.hpp"

DelayedSpikingNeuron2::DelayedSpikingNeuron2(int x, int y, xt::xarray<double> weights, xt::xarray<long> delays, double threshold) {
    m_events = std::vector<std::vector<Event>>(1000, std::vector<Event>());

    m_x = x;
    m_y = y;
    m_weights = std::move(weights);
    m_delays = std::move(delays);
    m_threshold = threshold;
    m_potential = 0.f;
    m_timestampLastEvent = 0;
    m_updateCount = 0;
    m_spike = false;
}

long DelayedSpikingNeuron2::getDelay(const int x, const int y) {
    return m_delays(y, x);
}

double DelayedSpikingNeuron2::getPotential(const long time) {
    return potentialDecay(time - m_timestampLastEvent);
}

void DelayedSpikingNeuron2::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    m_events[(m_updateCount + m_delays(y, x) - 1) % SPEED].emplace_back(timestamp, x, y, polarity);
}

void DelayedSpikingNeuron2::update(long time) {
    for (Event &event : m_events[m_updateCount]) {
        long dt_event = event.timestamp() - m_timestampLastEvent;
        m_potential = potentialDecay(dt_event);
        m_timestampLastEvent = event.timestamp();

        m_potential += m_weights(event.polarity(), event.y(), event.x());

        if (m_potential > m_threshold) {
            spike();
            break;
        }
    }
    m_events[m_updateCount].clear();
    m_updateCount++;
    if (m_updateCount >= SPEED) {
        m_updateCount = 0;
    }
}

double DelayedSpikingNeuron2::potentialDecay(const long time) {
    double new_potential = m_potential - static_cast<double>(time) * TAU_M;
    if (new_potential > 0) {
        return new_potential;
    }
    return 0;
}

void DelayedSpikingNeuron2::spike() {
    m_events = std::vector<std::vector<Event>>(1000, std::vector<Event>());
    m_potential = 0;
    m_spike = true;
}