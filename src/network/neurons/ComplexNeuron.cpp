#include "ComplexNeuron.hpp"

ComplexNeuron::ComplexNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, Eigen::Tensor<double, 3> &weights) :
    Neuron(index, layer, conf, pos, offset),
    m_events(boost::circular_buffer<NeuronEvent>(1000)),
    m_weights(weights) {
}

inline bool ComplexNeuron::newEvent(NeuronEvent event) {
    m_events.push_back(event);
    return membraneUpdate(event);
}

inline bool ComplexNeuron::membraneUpdate(NeuronEvent event) {
    m_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / conf.TAU_M);
//    potentialDecay(event.timestamp() - m_timestampLastEvent);

    m_potential += m_weights(event.x(), event.y(), event.z())
                   - refractoryPotential(event.timestamp() - m_spikingTime)
                   - m_adaptationPotential;
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
    ++m_spikeRateCounter;
    ++m_totalSpike;
    m_potential = conf.VRESET;

    spikeRateAdaptation();

    if (conf.TRACKING == "partial") {
        m_trackingSpikeTrain.push_back(time);
    }
}

inline void ComplexNeuron::weightUpdate() {
    if (conf.STDP_LEARNING) {
        for (NeuronEvent &event : m_events) {
            if (static_cast<double>(m_spikingTime - event.timestamp()) < conf.TAU_LTP) {
                m_weights(event.x(), event.y(), event.z()) += conf.ETA_LTP;
            }
            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) < conf.TAU_LTD) {
                m_weights(event.x(), event.y(), event.z()) += conf.ETA_LTD;
            }
            // Step Window
            //        if (m_networkConf.STDP == "step_sym") {
            //            if (static_cast<double>(m_spikingTime - event.timestamp()) < m_networkConf.TAU_LTP && static_cast<double>(m_spikingTime - event.timestamp()) >= 0) {
            //                m_weights(event.m_jitterSpeed(), event.y(), event.z()) += m_learningDecay * m_networkConf.ETA_LTP;
            //            }
            //            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) < m_networkConf.TAU_LTD && static_cast<double>(event.timestamp() - m_lastSpikingTime) >= 0) {
            //                m_weights(event.m_jitterSpeed(), event.y(), event.z()) += m_learningDecay * m_networkConf.ETA_LTD;
            //            }
            //        } else if (m_networkConf.STDP == "step_left") {
            //            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) < m_networkConf.TAU_LTD && static_cast<double>(event.timestamp() - m_lastSpikingTime) >= 0) {
            //                m_weights(event.m_jitterSpeed(), event.y(), event.z()) += m_learningDecay * m_networkConf.ETA_LTD;
            //            }
            //        } else if (m_networkConf.STDP == "lin_sym") {
            //            if (static_cast<double>(m_spikingTime - event.timestamp()) < m_networkConf.TAU_LTP  && static_cast<double>(m_spikingTime - event.timestamp()) >= 0) {
            //                m_weights(event.m_jitterSpeed(), event.y(), event.z()) += m_learningDecay * m_networkConf.ETA_LTP * (1 - static_cast<double>(m_spikingTime - event.timestamp()));
            //            }
            //            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) < m_networkConf.TAU_LTD && static_cast<double>(event.timestamp() - m_lastSpikingTime) >= 0) {
            //                m_weights(event.m_jitterSpeed(), event.y(), event.z()) += m_learningDecay * m_networkConf.ETA_LTD * (1 - static_cast<double>(event.timestamp() - m_lastSpikingTime));
            //            }
            //        } else if (m_networkConf.STDP == "exp_sym") {
            //            if (static_cast<double>(m_spikingTime - event.timestamp()) >= 0) {
            //                m_weights(event.m_jitterSpeed(), event.y(), event.z()) += m_learningDecay * m_networkConf.ETA_LTP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / m_networkConf.TAU_LTP);
            //            }
            //            if (static_cast<double>(event.timestamp() - m_lastSpikingTime) >= 0) {
            //                m_weights(event.m_jitterSpeed(), event.y(), event.z()) += m_learningDecay * m_networkConf.ETA_LTD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / m_networkConf.TAU_LTD);
            //            }
            //        }

            if (m_weights(event.x(), event.y(), event.z()) < 0) {
                m_weights(event.x(), event.y(), event.z()) = 0;
            }
        }

        normalizeWeights();
        //    m_learningDecay = 1 / (1 + exp(m_totalSpike - m_networkConf.DECAY_FACTOR));
    }
    m_events.clear();
}

inline void ComplexNeuron::normalizeWeights() {
    Eigen::Tensor<double, 0> norm = m_weights.pow(2).sum().sqrt();

    if (norm(0) != 0) {
        m_weights = conf.NORM_FACTOR * m_weights / norm(0);
    }
}

inline cv::Mat ComplexNeuron::summedWeightMatrix() {
    Eigen::Tensor<double, 2> sum = m_weights.sum(Eigen::array<double, 1>{2});
    const Eigen::Tensor<long, 2>::Dimensions &dim = sum.dimensions();
    Eigen::Tensor<double, 0> max = sum.maximum();
    Eigen::Tensor<double, 2> result = sum * 255. / max(0);

    cv::Mat mat = cv::Mat::zeros(static_cast<int>(dim[1]), static_cast<int>(dim[0]), CV_8UC3);
    for (int i = 0; i < dim[0]; ++i) {
        for (int j = 0; j < dim[1]; ++j) {
            auto &color = mat.at<cv::Vec3b>(j, i);
            color[0] = static_cast<unsigned char>(result(i, j));
        }
    }
    return mat;
}

void ComplexNeuron::saveWeights(std::string &saveFile) {
    Util::saveComplexTensorToNumpyFile(m_weights, saveFile);
}

void ComplexNeuron::loadWeights(std::string &filePath) {
    Util::loadNumpyFileToComplexTensor(filePath, m_weights);
}

std::vector<long> ComplexNeuron::getWeightsDimension() {
    const Eigen::Tensor<double, COMPLEXDIM>::Dimensions& dimensions = m_weights.dimensions();
    std::vector<long> dim = { dimensions[0], dimensions[1], dimensions[2] };
    return dim;
}