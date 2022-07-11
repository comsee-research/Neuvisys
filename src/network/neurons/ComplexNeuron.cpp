//
// Created by Thomas on 14/04/2021.
//

#include "ComplexNeuron.hpp"

ComplexNeuron::ComplexNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, const std::vector<size_t> &dimensions) :
        Neuron(index, layer, conf, pos, offset),
        m_events(boost::circular_buffer<NeuronEvent>(1000)) {
    m_weights = WeightMatrix(dimensions, true, m_conf.NORM_FACTOR);
}

inline bool ComplexNeuron::newEvent(NeuronEvent event) {
    m_events.push_back(event);
    return membraneUpdate(event);
}

inline bool ComplexNeuron::membraneUpdate(NeuronEvent event) {
    potentialDecay(event.timestamp());
    m_potential += m_weights.at(event.id())
                   - refractoryPotential(event.timestamp());
    m_timestampLastEvent = event.timestamp();
    if (m_potential > m_threshold) {
        spike(event.timestamp());
        return true;
    }
    return false;
}

inline void ComplexNeuron::spike(const size_t time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_spikeRateCounter;
    ++m_totalSpike;
    m_potential = m_conf.VRESET;

    spikeRateAdaptation();
    if (m_conf.TRACKING == "partial") {
        m_trackingSpikeTrain.push_back(time);
    }
}

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

inline cv::Mat ComplexNeuron::summedWeightMatrix() {
//    Eigen::Tensor<double, 2> sum = m_weights.sum(Eigen::array<double, 1>{2});
//    const Eigen::Tensor<long, 2>::Dimensions &dim = sum.dimensions();
//    Eigen::Tensor<double, 0> max = sum.maximum();
//    Eigen::Tensor<double, 2> result = sum * 255. / max(0);
//
//    cv::Mat mat = cv::Mat::zeros(static_cast<int>(dim[1]), static_cast<int>(dim[0]), CV_8UC3);
//    for (int i = 0; i < dim[0]; ++i) {
//        for (int j = 0; j < dim[1]; ++j) {
//            auto &color = mat.at<cv::Vec3b>(j, i);
//            color[0] = static_cast<unsigned char>(result(i, j));
//        }
//    }
    return cv::Mat();
}

void ComplexNeuron::saveWeights(const std::string &filePath) {
    auto weightsFile = filePath + std::to_string(m_index);
    m_weights.saveWeightsToNumpyFile(weightsFile);
}

void ComplexNeuron::loadWeights(std::string &filePath) {
    auto numpyFile = filePath + std::to_string(m_index) + ".npy";
    m_weights.loadNumpyFile(numpyFile);
}

void ComplexNeuron::loadWeights(cnpy::npz_t &arrayNPZ) {
    auto arrayName = std::to_string(m_index);
    m_weights.loadNumpyFile(arrayNPZ, arrayName);
}
