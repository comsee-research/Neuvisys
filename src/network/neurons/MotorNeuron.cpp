//
// Created by Thomas on 23/05/2021.
//

#include "MotorNeuron.hpp"

MotorNeuron::MotorNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, const std::vector<std::vector<size_t>> &dimensions) :
        Neuron(index, layer, conf, pos, Position()),
        m_events(boost::circular_buffer<NeuronEvent>(1000)) {
    size_t lay = 0;
    for (const auto &dimension : dimensions) {
        m_multiWeights.emplace_back(dimension);
        m_eligibilityTrace.emplace_back(dimension);
        m_eligibilityTiming.emplace_back(dimension);

        size_t nbWeights = 1;
        for (const auto d : dimension) {
            nbWeights *= d;
        }
        for (size_t i = 0; i < nbWeights; ++i) {
            m_multiWeights[lay].addWeight(i, true);
            m_eligibilityTrace[lay].addWeight(i, false);
            m_eligibilityTiming[lay].addWeight(i, false);
        }
        m_multiWeights[lay].normalize(m_conf.NORM_FACTOR);
        ++lay;
    }
}

inline bool MotorNeuron::newEvent(NeuronEvent event) {
    m_events.push_back(event);
    return membraneUpdate(event);
}

inline bool MotorNeuron::membraneUpdate(NeuronEvent event) {
    potentialDecay(event.timestamp());
    m_potential += m_multiWeights[event.layer()].at(event.id());
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
        if (count > 10000) {
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

inline cv::Mat MotorNeuron::summedWeightMatrix() {
    auto dim = m_multiWeights[0].getDimensions(); // TODO: displaying more than first connections

    cv::Mat mat = cv::Mat::zeros(static_cast<int>(dim[1]), static_cast<int>(dim[0]), CV_8UC3);
    double sum = 0, max = 0;
    for (int i = 0; i < dim[0]; ++i) {
        for (int j = 0; j < dim[1]; ++j) {
            for (int k = 0; k < dim[2]; ++k) {
                sum += m_multiWeights[0].at(k + j * dim[2] + i * dim[2] * dim[1]);
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

void MotorNeuron::saveWeights(const std::string &filePath) {
    size_t count = 0;
    for (auto &weights : m_multiWeights) {
        auto weightsFile = filePath + std::to_string(m_index) + "_" + std::to_string(count);
        weights.saveWeightsToNumpyFile(weightsFile);
        ++count;
    }
}

void MotorNeuron::loadWeights(std::string &filePath) {
    size_t count = 0;
    for (auto &weights : m_multiWeights) {
        auto numpyFile = filePath + std::to_string(m_index) + "_" + std::to_string(count) + ".npy";
        weights.saveWeightsToNumpyFile(numpyFile);
        ++count;
    }
}

void MotorNeuron::loadWeights(cnpy::npz_t &arrayNPZ) {
    size_t count = 0;
    for (auto &weights : m_multiWeights) {
        auto arrayName = std::to_string(m_index) + "_" + std::to_string(count);
        weights.loadNumpyFile(arrayNPZ, arrayName);
        ++count;
    }
}

inline void MotorNeuron::setNeuromodulator(double neuromodulator) {
    for (size_t i = 0; i < m_multiWeights.size(); ++i) {
        for (size_t j = 0; j < m_multiWeights[i].getSize(); ++j) {
            m_multiWeights[i].at(j) += m_conf.ETA * neuromodulator * m_eligibilityTrace[i].at(j);
            m_eligibilityTrace[i].at(j) = 0;
            if (m_multiWeights[i].at(j) < 0) {
                m_multiWeights[i].at(j) = 0;
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
