#include "OrientedSpikingNeuron.hpp"

OrientedSpikingNeuron::OrientedSpikingNeuron() = default;

OrientedSpikingNeuron::OrientedSpikingNeuron(const int x, const int y) : m_x(x), m_y(y) {
    m_potential = 0.f;
    m_timestampLastEvent = 0;
    m_weightsOn = xt::ones<double>({HEIGHT, WIDTH});
    m_weightsOff = xt::ones<double>({HEIGHT, WIDTH});
}

OrientedSpikingNeuron::OrientedSpikingNeuron(int x, int y, xt::xarray<double> weightsOn, xt::xarray<double> weightsOff, double threshold) : m_x(x), m_y(y), m_weightsOn(std::move(weightsOn)), m_weightsOff(std::move(weightsOff)), m_threshold(threshold) {
    m_potential = 0.f;
    m_timestampLastEvent = 0;
}

int OrientedSpikingNeuron::getX() {
    return m_x;
}

int OrientedSpikingNeuron::getY() {
    return m_y;
}

double OrientedSpikingNeuron::getThreshold() {
    return m_threshold;
}

double OrientedSpikingNeuron::getPotential() {
    return m_potential;
}

double OrientedSpikingNeuron::getPotential(const long time) {
    m_potential = potentialDecay(time - m_timestampLastEvent);
    return m_potential;
}

void OrientedSpikingNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    long dt_event = timestamp - m_timestampLastEvent;
    m_potential = potentialDecay(dt_event);
    m_timestampLastEvent = timestamp;

    if (polarity) {
        m_potential += m_weightsOn(y, x);
    } else {
        m_potential += m_weightsOff(y, x);
    }

    if (m_potential > m_threshold) {
        //return fire();
    }
}

double OrientedSpikingNeuron::potentialDecay(const long time) {
    double new_potential = m_potential - static_cast<double>(time) * DECAY;
    if (new_potential > 0) {
        return new_potential;
    }
    return 0;
}

bool OrientedSpikingNeuron::fire() {
    m_potential = 0;
    return true;
}