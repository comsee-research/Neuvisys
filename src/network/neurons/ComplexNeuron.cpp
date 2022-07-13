//
// Created by Thomas on 14/04/2021.
//

#include "ComplexNeuron.hpp"

/**
 *
 * @param index
 * @param layer
 * @param conf
 * @param pos
 * @param offset
 * @param dimensions
 */
ComplexNeuron::ComplexNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, const std::vector<size_t> &dimensions) :
        Neuron(index, layer, conf, pos, offset),
        m_events(boost::circular_buffer<NeuronEvent>(1000)) {
    m_weights = WeightMap(dimensions);
}

/**
 *
 * @param event
 * @return
 */
inline bool ComplexNeuron::newEvent(NeuronEvent event) {
    m_events.push_back(event);
    return membraneUpdate(event);
}

/**
 *
 * @param event
 * @return
 */
inline bool ComplexNeuron::membraneUpdate(NeuronEvent event) {
    potentialDecay(event.timestamp());
    m_potential += m_weights.at(event.id())
                   - refractoryPotential(event.timestamp());
    m_timestampLastEvent = event.timestamp();
    checkNegativeLimits();
    if (m_potential > m_threshold) {
        spike(event.timestamp());
        return true;
    }
    return false;
}

/**
 *
 * @param time
 */
inline void ComplexNeuron::spike(const size_t time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_spikeRateCounter;
    ++m_totalSpike;
    m_spikingPotential = m_potential;
    m_potential = m_conf.VRESET;

    spikeRateAdaptation();
    if (m_conf.TRACKING == "partial") {
        m_trackingSpikeTrain.push_back(time);
    }
}

/**
 *
 */
inline void ComplexNeuron::weightUpdate() {
    if (m_conf.STDP_LEARNING == "excitatory" || m_conf.STDP_LEARNING == "all") {
        for (NeuronEvent &event: m_events) {
            if (static_cast<double>(m_spikingTime - event.timestamp()) < m_conf.TAU_LTP) {
                m_weights.at(event.id()) += m_conf.ETA_LTP;
            }
            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) < m_conf.TAU_LTD) {
                m_weights.at(event.id()) += m_conf.ETA_LTD;
            }
            if (m_weights.at(event.id()) < 0) {
                m_weights.at(event.id()) = 0;
            }
        }

        m_weights.normalize(m_conf.NORM_FACTOR);
        //    m_decay = 1 / (1 + exp(m_totalSpike - m_networkConf.DECAY_FACTOR));
    }
    m_events.clear();
}

/**
 *
 * @return
 */
inline cv::Mat ComplexNeuron::summedWeightMatrix() {
    auto dim = m_weights.getDimensions();

    cv::Mat mat = cv::Mat::zeros(static_cast<int>(dim[1]), static_cast<int>(dim[0]), CV_8UC3);
    double sum = 0, max = 0;
    for (int i = 0; i < dim[0]; ++i) {
        for (int j = 0; j < dim[1]; ++j) {
            for (int k = 0; k < dim[2]; ++k) {
                sum += m_weights.at(k + j * dim[2] + i * dim[2] * dim[1]);
            }
            if (sum > max) {
                max = sum;
            }
            auto &color = mat.at<cv::Vec3b>(j, i);
            color[0] = static_cast<unsigned char>(sum);
            sum = 0;
        }
    }
    mat = mat * 255.0 / max;
    return mat;
}

/**
 *
 * @param filePath
 */
void ComplexNeuron::saveWeights(const std::string &filePath) {
    auto weightsFile = filePath + std::to_string(m_index);
    m_weights.saveWeightsToNumpyFile(weightsFile);
}

/**
 *
 * @param filePath
 */
void ComplexNeuron::loadWeights(std::string &filePath) {
    auto numpyFile = filePath + std::to_string(m_index) + ".npy";
    m_weights.loadNumpyFile(numpyFile);
}

/**
 *
 * @param arrayNPZ
 */
void ComplexNeuron::loadWeights(cnpy::npz_t &arrayNPZ) {
    auto arrayName = std::to_string(m_index);
    m_weights.loadNumpyFile(arrayNPZ, arrayName);
}
