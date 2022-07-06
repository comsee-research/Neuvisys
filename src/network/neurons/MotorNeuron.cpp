//
// Created by Thomas on 23/05/2021.
//

#include "MotorNeuron.hpp"

MotorNeuron::MotorNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, const std::vector<std::vector<size_t>> &dimensions) :
        Neuron(index, layer, conf, pos, Position()),
        m_events(boost::circular_buffer<NeuronEvent>(1000)) {
    for (size_t i = 0; i < dimensions.size(); ++i) {
        m_weights[i] = WeightMatrix(dimensions[i], true);
        m_weights[i].normalize(m_conf.NORM_FACTOR);
        m_eligibilityTrace[i] = WeightMatrix(dimensions[i]);
        m_eligibilityTiming[i] = WeightMatrix(dimensions[i]);
    }
}

inline bool MotorNeuron::newEvent(NeuronEvent event) {
    m_events.push_back(event);
    return membraneUpdate(event);
}

inline bool MotorNeuron::membraneUpdate(NeuronEvent event) {
    potentialDecay(event.timestamp());
    m_potential += m_weights[event.layer()].at(event.id());
    m_timestampLastEvent = event.timestamp();

    if (m_potential > m_threshold) {
        spike(event.timestamp());
        return true;
    }
    return false;
}

inline void MotorNeuron::spike(size_t time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_spikeRateCounter;
    ++m_activityCounter;
    ++m_totalSpike;
    m_potential = m_conf.VRESET;
    m_trackingSpikeTrain.push_back(time);
}

inline void MotorNeuron::weightUpdate() {
    if (m_conf.STDP_LEARNING == "excitatory" || m_conf.STDP_LEARNING == "all") {
        for (NeuronEvent &event: m_events) {
            m_eligibilityTrace[event.layer()].at(event.id()) *= exp(
                    -(static_cast<double>(event.timestamp()) - m_eligibilityTiming[event.layer()].at(event.id())) / m_conf.TAU_E);
            m_eligibilityTrace[event.layer()].at(event.id()) +=
                    m_conf.ETA_LTP * exp(-static_cast<double>(m_spikingTime - event.timestamp()) / m_conf.TAU_LTP);
            m_eligibilityTrace[event.layer()].at(event.id()) +=
                    m_conf.ETA_LTD * exp(-static_cast<double>(event.timestamp() - m_lastSpikingTime) / m_conf.TAU_LTD);
            if (m_eligibilityTrace[event.layer()].at(event.id()) < 0) {
                m_eligibilityTrace[event.layer()].at(event.id()) = 0;
            }
            m_eligibilityTiming[event.layer()].at(event.id()) = static_cast<double>(event.timestamp());
        }
    }
    m_events.clear();
}

double MotorNeuron::updateKernelSpikingRate(long time) {
    double kernelSpikingRate = 0;
    size_t count = 0;
    for (auto spikeTime = m_trackingSpikeTrain.rbegin(); spikeTime != m_trackingSpikeTrain.rend(); ++spikeTime) {
        if (count > 200) {
            break;
        } else {
            kernelSpikingRate += kernel(static_cast<double>(time - *spikeTime) / E6);
            ++count;
        }
    }
    return kernelSpikingRate;
}

inline double MotorNeuron::kernel(double time) {
    return (exp(-time / m_conf.TAU_K) - exp(-time / m_conf.NU_K)) / (m_conf.TAU_K - m_conf.NU_K);
}

//inline double MotorNeuron::kernelDerivative(double time) {
//    return (exp(-time / m_conf.NU_K) / m_conf.NU_K - exp(-time / m_conf.TAU_K) / m_conf.TAU_K) / (m_conf.TAU_K - m_conf.NU_K);
//}

//inline cv::Mat MotorNeuron::summedWeightMatrix() {
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
//    return mat;
//}

void MotorNeuron::saveWeights(std::string &filePath) {
    size_t count = 0;
    for (const auto &weights : m_weights) {
        auto weightsFile = filePath + std::to_string(m_index) + "_" + std::to_string(count);
        Util::saveWeightsToNumpyFile(weights, weightsFile);
        ++count;
    }
}

void MotorNeuron::loadWeights(std::string &filePath) {
    size_t count = 0;
    for (auto &weights : m_weights) {
        auto numpyFile = filePath + std::to_string(m_index) + "_" + std::to_string(count) + ".npy";
        Util::loadNumpyFileToWeights(weights, numpyFile);
        ++count;
    }
}

//void MotorNeuron::loadWeights(cnpy::npz_t &arrayNPZ) {
//    auto arrayName = std::to_string(m_index);
//    Util::loadNumpyFileToWeights(m_weights, arrayNPZ, arrayName);
//}

double MotorNeuron::getWeights(long x, long y, long z) {
    return m_weights.at(x);
}

//std::vector<long> MotorNeuron::getWeightsDimension() {
//    const Eigen::Tensor<double, COMPLEXDIM>::Dimensions &dimensions = m_weights.dimensions();
//    std::vector<long> dim = {dimensions[0], dimensions[1], dimensions[2]};
//    return dim;
//}

inline void MotorNeuron::setNeuromodulator(double neuromodulator) {
    for (size_t i = 0; i < m_weights.size(); ++i) {
        for (size_t j = 0; j < m_weights[i].getSize(); ++i) {
            m_weights[i].at(j) += m_conf.ETA * neuromodulator * m_eligibilityTrace[i].at(j);
            m_eligibilityTrace[i].at(j) = 0;
            if (m_weights[i].at(j) < 0) {
                m_weights[i].at(j) = 0;
            }
        }
    }
}

void MotorNeuron::learningDecay(double decay) {
    m_conf.ETA *= decay;

    if (m_conf.TAU_K > m_conf.MIN_TAU_K) {
        m_conf.TAU_K *= decay;
    }
    if (m_conf.NU_K > m_conf.MIN_NU_K) {
        m_conf.NU_K *= decay;
    }
}
