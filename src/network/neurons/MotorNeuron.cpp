//
// Created by alphat on 23/05/2021.
//

#include "MotorNeuron.hpp"

MotorNeuron::MotorNeuron(size_t index, NeuronConfig &conf, Position pos, Eigen::Tensor<double, 3> &weights) :
    Neuron(index, conf, pos, Position()),
    m_events(boost::circular_buffer<NeuronEvent>(1000)),
    m_weights(weights) {
}

inline bool MotorNeuron::newEvent(NeuronEvent event, double reward) {
    m_reward = reward;
    m_events.push_back(event);
    return membraneUpdate(event);
}

inline bool MotorNeuron::membraneUpdate(NeuronEvent event) {
    m_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / conf.TAU_M);
    m_potential += m_weights(event.x(), event.y(), event.z());
    m_timestampLastEvent = event.timestamp();

    if (m_potential > m_threshold) {
        spike(event.timestamp());
        return true;
    }
    return false;
}

inline void MotorNeuron::spike(long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_countSpike;
    ++m_totalSpike;
    m_potential = conf.VRESET;

    if (conf.STDP_LEARNING) {
        updateSTDP();
    }
    m_events.clear();

    if (conf.TRACKING == "partial") {
        m_trackingSpikeTrain.push_back(time);
    }
}

inline void MotorNeuron::updateSTDP() {
    for (NeuronEvent &event : m_events) {
        if (static_cast<double>(m_spikingTime - event.timestamp()) >= 0) {
            m_weights(event.x(), event.y(), event.z()) += m_reward * m_learningDecay * conf.ETA_LTP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / conf.TAU_LTP);
        }
        if (static_cast<double>(event.timestamp() - m_lastSpikingTime) >= 0) {
            m_weights(event.x(), event.y(), event.z()) += m_reward * m_learningDecay * conf.ETA_LTD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / conf.TAU_LTD);
        }

        if (m_weights(event.x(), event.y(), event.z()) < 0) {
            m_weights(event.x(), event.y(), event.z()) = 0;
        }
    }

    normalizeWeights();
}

inline void MotorNeuron::normalizeWeights() {
    Eigen::Tensor<double, 0> norm = m_weights.pow(2).sum().sqrt();

    if (norm(0) != 0) {
        m_weights = conf.NORM_FACTOR * m_weights / norm(0);
    }
}

void MotorNeuron::saveWeights(std::string &saveFile) {
    Util::saveComplexTensorToNumpyFile(m_weights, saveFile);
}

void MotorNeuron::loadWeights(std::string &filePath) {
    Util::loadNumpyFileToComplexTensor(filePath, m_weights);
}