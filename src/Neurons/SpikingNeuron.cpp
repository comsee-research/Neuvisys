#include "SpikingNeuron.hpp"
#include "math.h"

inline int SpikingNeuron::getX() {
    return m_x;
}

inline int SpikingNeuron::getY() {
    return m_y;
}

inline double SpikingNeuron::getWeightsOn(const int x, const int y) {
    return m_weightsOn(y, x);
}

inline double SpikingNeuron::getWeightsOff(const int x, const int y) {
    return m_weightsOff(y, x);
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

inline void SpikingNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {

}

inline bool SpikingNeuron::fire() {
    m_potential = VRESET;
    return true;
}
