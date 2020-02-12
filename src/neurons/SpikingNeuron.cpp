#include "SpikingNeuron.hpp"
#include "math.h"

inline int SpikingNeuron::getX() {
    return m_x;
}

inline int SpikingNeuron::getY() {
    return m_y;
}

inline double SpikingNeuron::getWeights(const int p, const int x, const int y) {
    return m_weights(p, y, x);
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
    return m_potential * exp(- static_cast<double>(time) / TAU_M);
}

inline bool SpikingNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {

}

inline void SpikingNeuron::spike() {
    m_potential = VRESET;
    m_spike = true;
}

bool SpikingNeuron::hasSpiked() {
    if (m_spike){
        m_spike = false;
        return true;
    }
    return false;
}

void SpikingNeuron::setInhibitionTime(long inhibitionTime) {
    m_inhibitionTime = inhibitionTime;
}
