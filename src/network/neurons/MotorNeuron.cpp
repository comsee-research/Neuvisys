//
// Created by alphat on 23/05/2021.
//

#include "MotorNeuron.hpp"

MotorNeuron::MotorNeuron(size_t index, NeuronConfig &conf, Position pos, Eigen::Tensor<double, 3> &weights) :
    Neuron(index, conf, pos, Position()),
    m_events(boost::circular_buffer<NeuronEvent>(1000)),
    m_weights(weights) {
    const Eigen::Tensor<double, COMPLEXDIM>::Dimensions &d = m_weights.dimensions();
    m_eligibilityTrace = Eigen::Tensor<double, COMPLEXDIM>(d[0], d[1], d[2]);
    for (long i = 0; i < d[0]; ++i) {
        for (long j = 0; j < d[1]; ++j) {
            for (long k = 0; k < d[2]; ++k) {
                m_eligibilityTrace(i, j, k) = 0;
            }
        }
    }
}

inline bool MotorNeuron::newEvent(NeuronEvent event) {
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

    if (conf.TRACKING == "partial") {
        m_trackingSpikeTrain.push_back(time);
    }
}

inline void MotorNeuron::weightUpdate() {
    if (conf.STDP_LEARNING) {
        for (NeuronEvent &event : m_events) {
            m_eligibilityTrace(event.x(), event.y(), event.z()) *= exp(- static_cast<double>(m_spikingTime - m_lastSpikingTime) / conf.TAU_E);

            m_eligibilityTrace(event.x(), event.y(), event.z()) += conf.ETA_LTP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / conf.TAU_LTP);
            m_eligibilityTrace(event.x(), event.y(), event.z()) += conf.ETA_LTD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / conf.TAU_LTD);
            if (m_eligibilityTrace(event.x(), event.y(), event.z()) < 0) {
                m_eligibilityTrace(event.x(), event.y(), event.z()) = 0;
            }

            m_weights(event.x(), event.y(), event.z()) += m_reward * m_eligibilityTrace(event.x(), event.y(), event.z());

            if (m_weights(event.x(), event.y(), event.z()) < 0) {
                m_weights(event.x(), event.y(), event.z()) = 0;
            }
        }

        normalizeWeights();
    }
    m_events.clear();
}

inline void MotorNeuron::normalizeWeights() {
    Eigen::Tensor<double, 0> normT = m_weights.pow(2).sum().sqrt();
    double norm = normT(0);

    if (norm != 0) {
        m_weights = conf.NORM_FACTOR * m_weights / norm;
    }
}

void MotorNeuron::saveWeights(std::string &saveFile) {
    Util::saveComplexTensorToNumpyFile(m_weights, saveFile);
}

void MotorNeuron::loadWeights(std::string &filePath) {
    Util::loadNumpyFileToComplexTensor(filePath, m_weights);
}

double MotorNeuron::getWeights(long x, long y, long z) {
    return m_weights(x, y, z);
}

std::vector<long> MotorNeuron::getWeightsDimension() {
    const Eigen::Tensor<double, COMPLEXDIM>::Dimensions& dimensions = m_weights.dimensions();
    std::vector<long> dim = { dimensions[0], dimensions[1], dimensions[2] };
    return dim;
}

inline void MotorNeuron::setReward(double reward, double bias) {
    m_reward = reward;
    m_bias = bias;
}