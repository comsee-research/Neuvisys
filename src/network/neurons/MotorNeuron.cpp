//
// Created by alphat on 23/05/2021.
//

#include "MotorNeuron.hpp"

MotorNeuron::MotorNeuron(size_t index, NeuronConfig &conf, Position pos, Position offset) : Neuron(index, conf, pos, offset) {

}

inline bool MotorNeuron::newEvent(NeuronEvent event) {
//    m_events.push_back(event);
    return membraneUpdate(event);
}

void MotorNeuron::spike(long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_countSpike;
    ++m_totalSpike;
    m_potential = conf.VRESET;

//    m_events.clear();

    if (conf.TRACKING == "partial") {
        m_trackingSpikeTrain.push_back(time);
    }
}

bool MotorNeuron::membraneUpdate(NeuronEvent event) {
    m_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / conf.TAU_M);
    m_potential += 2;
    m_timestampLastEvent = event.timestamp();

    if (m_potential > m_threshold) {
        spike(event.timestamp());
        return true;
    }
    return false;
}
