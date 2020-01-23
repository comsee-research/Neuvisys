#include "DelayedSpikingNeuron2.hpp"

DelayedSpikingNeuron2::DelayedSpikingNeuron2(int x, int y, xt::xarray<double> weightsOn, xt::xarray<double> weightsOff, xt::xarray<long> delays, double threshold) {
    m_events = std::vector<std::vector<Event>>(1000, std::vector<Event>(0));
    m_x = x;
    m_y = y;
    m_weightsOn = std::move(weightsOn);
    m_weightsOff = std::move(weightsOff);
    m_delays = std::move(delays);
    m_threshold = threshold;
    m_potential = 0.f;
    m_timestampLastEvent = 0;
    m_updateCount = 0;
}

long DelayedSpikingNeuron2::getDelay(const int x, const int y) {
    return m_delays(y, x);
}

double DelayedSpikingNeuron2::getPotential(const long time) {
    return potentialDecay(time - m_timestampLastEvent);
}

void DelayedSpikingNeuron2::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    m_events[(m_updateCount + m_delays(y, x)) % SPEED].emplace_back(timestamp, x, y, polarity);
}

bool DelayedSpikingNeuron2::update(long time) {
    for (Event &event : m_events[m_updateCount]) {
        long dt_event = event.timestamp() - m_timestampLastEvent;
        m_potential = potentialDecay(dt_event);
        m_timestampLastEvent = event.timestamp();

        if (event.polarity()) {
            m_potential += m_weightsOn(event.y(), event.x());
        } else {
            m_potential += m_weightsOff(event.y(), event.x());
        }

        if (m_potential > m_threshold) {
            //return fire();
        }
    }
    m_events[m_updateCount].clear();
    m_updateCount++;
    if (m_updateCount >= SPEED) {
        m_updateCount = 0;
    }
    return false;
}

double DelayedSpikingNeuron2::potentialDecay(const long time) {
    double new_potential = m_potential - static_cast<double>(time) * DECAY;
    if (new_potential > 0) {
        return new_potential;
    }
    return 0;
}

bool DelayedSpikingNeuron2::fire() {
    m_events = std::vector<std::vector<Event>>();
    m_potential = 0;
    return true;
}