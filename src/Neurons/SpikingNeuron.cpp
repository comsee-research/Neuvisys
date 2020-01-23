#include "SpikingNeuron.hpp"

int SpikingNeuron::getX() {
    return m_x;
}

int SpikingNeuron::getY() {
    return m_y;
}

double SpikingNeuron::getThreshold() {
    return m_threshold;
}

double SpikingNeuron::getPotential() {
    return m_potential;
}

double SpikingNeuron::getPotential(long time) {
    return 0;
}

double SpikingNeuron::potentialDecay(const long time) {
    double new_potential = m_potential - static_cast<double>(time) * DECAY;
    if (new_potential > 0) {
        return new_potential;
    }
    return 0;
}

void SpikingNeuron::newEvent(long timestamp, int x, int y, bool polarity) {

}

bool SpikingNeuron::fire() {
    return false;
}
