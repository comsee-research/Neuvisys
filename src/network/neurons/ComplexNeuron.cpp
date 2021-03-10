#include "ComplexNeuron.hpp"

ComplexNeuron::ComplexNeuron(size_t index, NeuronConfig &conf, Luts &luts, Position pos, Position offset, Eigen::Tensor<double, 3> &weights) :
    Neuron(index, conf, luts, pos, offset),
    m_events(boost::circular_buffer<NeuronEvent>(1000)),
    m_weights(weights) {
}

inline bool ComplexNeuron::newEvent(NeuronEvent event) {
    m_events.push_back(event);
    return membraneUpdate(event);
}

inline bool ComplexNeuron::membraneUpdate(NeuronEvent event) {
    if (event.timestamp() - m_timestampLastEvent < 1000000) {
        m_potential *= m_luts.lutM[static_cast<size_t>(event.timestamp() - m_timestampLastEvent)];
    } else {
        m_potential = 0;
    }
//    potentialDecay(event.timestamp() - m_timestampLastEvent);
    m_potential += m_weights(event.x(), event.y(), event.z())
                - refractoryPotential(event.timestamp() - m_spikingTime)
                - m_adaptation_potential;
    m_timestampLastEvent = event.timestamp();

    if (m_potential > m_threshold) {
        spike(event.timestamp());
        return true;
    }
    return false;
}

inline void ComplexNeuron::spike(const long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_countSpike;
    ++m_totalSpike;
    m_potential = conf.VRESET;

    spikeRateAdaptation();
    if (conf.STDP_LEARNING) {
        updateSTDP();
    }
    m_events.clear();

    if (conf.TRACKING == "full" || conf.TRACKING == "partial") {
        m_spikeTrain.push_back(time);
    }
}

inline void ComplexNeuron::updateSTDP() {
    for (NeuronEvent &event : m_events) {
        if (static_cast<double>(m_spikingTime - event.timestamp()) < conf.TAU_LTP) {
            m_weights(event.x(), event.y(), event.z()) += m_learningDecay * conf.ETA_LTP;
        }
        if (static_cast<double>(event.timestamp() - m_lastSpikingTime) < conf.TAU_LTD) {
            m_weights(event.x(), event.y(), event.z()) += m_learningDecay * conf.ETA_LTD;
        }
        // Step Window
//        if (m_conf.STDP == "step_sym") {
//            if (static_cast<double>(m_spikingTime - event.timestamp()) < m_conf.TAU_LTP && static_cast<double>(m_spikingTime - event.timestamp()) >= 0) {
//                m_weights(event.x(), event.y(), event.z()) += m_learningDecay * m_conf.ETA_LTP;
//            }
//            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) < m_conf.TAU_LTD && static_cast<double>(event.timestamp() - m_lastSpikingTime) >= 0) {
//                m_weights(event.x(), event.y(), event.z()) += m_learningDecay * m_conf.ETA_LTD;
//            }
//        } else if (m_conf.STDP == "step_left") {
//            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) < m_conf.TAU_LTD && static_cast<double>(event.timestamp() - m_lastSpikingTime) >= 0) {
//                m_weights(event.x(), event.y(), event.z()) += m_learningDecay * m_conf.ETA_LTD;
//            }
//        } else if (m_conf.STDP == "lin_sym") {
//            if (static_cast<double>(m_spikingTime - event.timestamp()) < m_conf.TAU_LTP  && static_cast<double>(m_spikingTime - event.timestamp()) >= 0) {
//                m_weights(event.x(), event.y(), event.z()) += m_learningDecay * m_conf.ETA_LTP * (1 - static_cast<double>(m_spikingTime - event.timestamp()));
//            }
//            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) < m_conf.TAU_LTD && static_cast<double>(event.timestamp() - m_lastSpikingTime) >= 0) {
//                m_weights(event.x(), event.y(), event.z()) += m_learningDecay * m_conf.ETA_LTD * (1 - static_cast<double>(event.timestamp() - m_lastSpikingTime));
//            }
//        } else if (m_conf.STDP == "exp_sym") {
//            if (static_cast<double>(m_spikingTime - event.timestamp()) >= 0) {
//                m_weights(event.x(), event.y(), event.z()) += m_learningDecay * m_conf.ETA_LTP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / m_conf.TAU_LTP);
//            }
//            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) >= 0) {
//                m_weights(event.x(), event.y(), event.z()) += m_learningDecay * m_conf.ETA_LTD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / m_conf.TAU_LTD);
//            }
//        }

        if (m_weights(event.x(), event.y(), event.z()) < 0) {
            m_weights(event.x(), event.y(), event.z()) = 0;
        }
    }

    normalizeWeights();
//    m_learningDecay = 1 / (1 + exp(m_totalSpike - m_conf.DECAY_FACTOR));
}

inline void ComplexNeuron::normalizeWeights() {
    Eigen::Tensor<double, 0> norm = m_weights.pow(2).sum().sqrt();

    if (norm(0) != 0) {
        m_weights = conf.NORM_FACTOR * m_weights / norm(0);
    }
}

void ComplexNeuron::saveWeights(std::string &saveFile) {
    Util::saveComplexTensorToNumpyFile(m_weights, saveFile);
}

void ComplexNeuron::loadWeights(std::string &filePath) {
    Util::loadNumpyFileToComplexTensor(filePath, m_weights);
}

double ComplexNeuron::getWeights(long x, long y, long z) {
    return m_weights(x, y, z);
}
