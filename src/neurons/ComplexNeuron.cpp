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
//    potentialDecay(timestamp - m_timestampLastEvent);
    if (event.timestamp() - m_timestampLastEvent < 1000000) {
        m_potential *= m_luts.lutM[static_cast<size_t>(event.timestamp() - m_timestampLastEvent)];
    } else {
        m_potential = 0;
    }
    m_potential += m_weights(event.x(), event.y(), event.z())
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

    if (conf.TRACKING) {
        m_spikeTrain.push_back(time);
    }
}

inline void ComplexNeuron::updateSTDP() {
    for (NeuronEvent &event : m_events) {
        if (static_cast<double>(m_spikingTime - event.timestamp()) < conf.TAU_LTP) {
            m_weights(event.x(), event.y(), event.z()) += m_learningDecay * conf.DELTA_VP;
        }

        if (m_weights(event.x(), event.y(), event.z()) < 0) {
            m_weights(event.x(), event.y(), event.z()) = 0;
        }
    }

    normalizeWeights();
//    m_learningDecay = 1 / (1 + exp(m_totalSpike - conf.DECAY_FACTOR));
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
