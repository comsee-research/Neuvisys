//
// Created by alphat on 23/05/2021.
//

#include "MotorNeuron.hpp"

MotorNeuron::MotorNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos,
                         Eigen::Tensor<double, 3> &weights) :
        Neuron(index, layer, conf, pos, Position()),
        m_events(boost::circular_buffer<NeuronEvent>(1000)),
        m_weights(weights) {
    const Eigen::Tensor<double, COMPLEXDIM>::Dimensions &d = m_weights.dimensions();
    m_eligibilityTrace = Eigen::Tensor<double, COMPLEXDIM>(d[0], d[1], d[2]);
    m_eligibilityTiming = Eigen::Tensor<double, COMPLEXDIM>(d[0], d[1], d[2]);
    for (long i = 0; i < d[0]; ++i) {
        for (long j = 0; j < d[1]; ++j) {
            for (long k = 0; k < d[2]; ++k) {
                m_eligibilityTrace(i, j, k) = 0;
                m_eligibilityTiming(i, j, k) = 0;
            }
        }
    }
}

inline bool MotorNeuron::newEvent(NeuronEvent event) {
    m_events.push_back(event);
    return membraneUpdate(event);
}

inline bool MotorNeuron::membraneUpdate(NeuronEvent event) {
    m_potential *= exp(-static_cast<double>(event.timestamp() - m_timestampLastEvent) / m_conf.TAU_M);
    m_potential += m_weights(event.x(), event.y(), event.z());
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
            m_eligibilityTrace(event.x(), event.y(), event.z()) *=
                    exp(-(static_cast<double>(event.timestamp()) - m_eligibilityTiming(event.x(), event.y(), event.z())) / m_conf.TAU_E);
            m_eligibilityTrace(event.x(), event.y(), event.z()) +=
                    m_conf.ETA_LTP * exp(-static_cast<double>(m_spikingTime - event.timestamp()) / m_conf.TAU_LTP);
            if (m_layer < 3) {
                m_eligibilityTrace(event.x(), event.y(), event.z()) +=
                        m_conf.ETA_LTD * exp(-static_cast<double>(event.timestamp() - m_lastSpikingTime) / m_conf.TAU_LTD);
            }
            if (m_eligibilityTrace(event.x(), event.y(), event.z()) < 0) {
                m_eligibilityTrace(event.x(), event.y(), event.z()) = 0;
            }
            m_eligibilityTiming(event.x(), event.y(), event.z()) = static_cast<double>(event.timestamp());

            m_weights(event.x(), event.y(), event.z()) +=
                    m_conf.ETA * m_neuromodulator * m_eligibilityTrace(event.x(), event.y(), event.z());
            if (m_weights(event.x(), event.y(), event.z()) < 0) {
                m_weights(event.x(), event.y(), event.z()) = 0;
            }
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

inline void MotorNeuron::normalizeWeights() {
    auto norm = computeNormWeights();
    if (norm != 0) {
        m_weights = m_conf.NORM_FACTOR * m_weights / norm;
    }
}

inline double MotorNeuron::computeNormWeights() {
    Eigen::Tensor<double, 0> normT = m_weights.pow(2).sum().sqrt();
    return normT(0);
}

inline void MotorNeuron::rescaleWeights(double scale) {
    m_weights = m_weights * scale;
}

inline cv::Mat MotorNeuron::summedWeightMatrix() {
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

void MotorNeuron::saveWeights(std::string &filePath) {
    auto weightFile = filePath + std::to_string(m_index);
    Util::saveComplexTensorToNumpyFile(m_weights, weightFile);
}

void MotorNeuron::loadWeights(std::string &filePath) {
    auto weightFile = filePath + std::to_string(m_index);
    Util::loadNumpyFileToComplexTensor(m_weights, weightFile);
}

double MotorNeuron::getWeights(long x, long y, long z) {
    return m_weights(x, y, z);
}

std::vector<long> MotorNeuron::getWeightsDimension() {
    const Eigen::Tensor<double, COMPLEXDIM>::Dimensions &dimensions = m_weights.dimensions();
    std::vector<long> dim = {dimensions[0], dimensions[1], dimensions[2]};
    return dim;
}

inline void MotorNeuron::setNeuromodulator(double neuromodulator) {
    m_neuromodulator = neuromodulator;
}

void MotorNeuron::learningDecay(double decay) {
    m_conf.ETA /= decay;

    if (m_conf.TAU_K > m_conf.MIN_TAU_K) {
        m_conf.TAU_K /= decay;
    }
    if (m_conf.NU_K > m_conf.MIN_NU_K) {
        m_conf.NU_K /= decay;
    }
}
