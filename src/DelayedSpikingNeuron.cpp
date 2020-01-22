#include "DelayedSpikingNeuron.hpp"

DelayedSpikingNeuron::DelayedSpikingNeuron() = default;

DelayedSpikingNeuron::DelayedSpikingNeuron(const int x, const int y) : m_x(x), m_y(y) {
    m_potential = 0.f;
    m_timestampLastEvent = 0;
    m_weightsOn = xt::ones<double>({HEIGHT, WIDTH});
    m_weightsOff = xt::ones<double>({HEIGHT, WIDTH});
    m_delays = xt::zeros<long>({HEIGHT, WIDTH});
}

DelayedSpikingNeuron::DelayedSpikingNeuron(int x, int y, xt::xarray<double> weightsOn, xt::xarray<double> weightsOff, xt::xarray<long> delays, double threshold) : m_x(x), m_y(y), m_weightsOn(std::move(weightsOn)), m_weightsOff(std::move(weightsOff)), m_delays(std::move(delays)), m_threshold(threshold) {
    m_potential = 0.f;
    m_timestampLastEvent = 0;
}

int DelayedSpikingNeuron::getX() {
    return m_x;
}

int DelayedSpikingNeuron::getY() {
    return m_y;
}

double DelayedSpikingNeuron::getThreshold() {
    return m_threshold;
}

long DelayedSpikingNeuron::getDelay(const int x, const int y) {
    return m_delays(y, x);
}

bool DelayedSpikingNeuron::isEmpty() {
    return m_events.empty();
}

long DelayedSpikingNeuron::getTimestampNextEvent() {
    return m_events.top().timestamp();
}

double DelayedSpikingNeuron::getPotential() {
    return m_potential;
}

double DelayedSpikingNeuron::getPotential(const long time) {
    return potentialDecay(time - m_timestampLastEvent);
}

void DelayedSpikingNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    m_events.emplace(timestamp + m_delays(y, x), x, y, polarity);
}

bool DelayedSpikingNeuron::update(long time) {
    while (!isEmpty() && getTimestampNextEvent() <= time) {
        Event event = m_events.top();
        m_events.pop();

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
    return false;
}

double DelayedSpikingNeuron::potentialDecay(const long time) {
    double new_potential = m_potential - static_cast<double>(time) * DECAY;
    if (new_potential > 0) {
        return new_potential;
    }
    return 0;
}

bool DelayedSpikingNeuron::fire() {
    m_events = std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>();
    m_potential = 0;
    return true;
}
