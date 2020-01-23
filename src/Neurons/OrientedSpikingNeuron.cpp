#include "OrientedSpikingNeuron.hpp"

OrientedSpikingNeuron::OrientedSpikingNeuron(int x, int y, xt::xarray<double> weightsOn, xt::xarray<double> weightsOff, double threshold) {
    m_x = x;
    m_y = y;
    m_weightsOn = std::move(weightsOn);
    m_weightsOff = std::move(weightsOff);
    m_threshold = threshold;
    m_potential = 0.f;
    m_timestampLastEvent = 0;
}

double OrientedSpikingNeuron::getPotential(const long time) {
    m_potential = potentialDecay(time - m_timestampLastEvent);
    return m_potential;
}

void OrientedSpikingNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    //std::lock_guard<std::mutex> guard(g_potential);
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

bool OrientedSpikingNeuron::fire() {
    m_potential = 0;
    return true;
}