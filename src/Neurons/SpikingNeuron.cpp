#include "SpikingNeuron.hpp"
#include "math.h"

int SpikingNeuron::getX() {
    return m_x;
}

int SpikingNeuron::getY() {
    return m_y;
}

double getWeightOn(int i, int j) {
    return m_weigthsOn
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

inline double SpikingNeuron::potentialDecay(const long time) {
    return m_potential * exp(- static_cast<double>(time) / DECAY);
}

void SpikingNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {

}

bool SpikingNeuron::fire() {
    m_potential = VRESET;
    return true;
}
