#include "SimpleNeuron.hpp"

/* Similar to the abstract neuron class.
 * It also takes as input a weight tensor and the number of synapses used when delayed synapses are defined.
 */
SimpleNeuron::SimpleNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, Eigen::Tensor<double, SIMPLEDIM> &weights, size_t nbSynapses) :
    Neuron(index, layer, conf, pos, offset),
    m_events(boost::circular_buffer<Event>(1000)),
    m_inhibEvents(boost::circular_buffer<NeuronEvent>(1000)),
    m_weights(weights),
    m_inhibWeights(Util::uniformMatrixComplex(1, 1, 16)), // TODO: generic init
    m_waitingList(std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>()) {
    for (size_t synapse = 0; synapse < nbSynapses; synapse++) {
        m_delays.push_back(static_cast<long>(synapse * conf.SYNAPSE_DELAY));
    }
}

/* Updates neuron internal state after the arrival of an event
 * Checks first if there is some synaptic delays defined in the network.
 */
inline bool SimpleNeuron::newEvent(Event event) {
    if ((m_delays.size() == 1) && (m_delays[0] == 0)) {
        m_events.push_back(event);
        return membraneUpdate(event);
    } else {
        size_t synapse = 0;
        for (auto delay : m_delays) {
            m_waitingList.emplace(event.timestamp() + delay, event.x(), event.y(), event.polarity(), event.camera(), synapse++);
        }
    }
}

void SimpleNeuron::newInhibitoryEvent(NeuronEvent event) {
    m_inhibEvents.push_back(event);
    m_potential -= m_inhibWeights(event.x(), event.y(), event.z());
}

bool SimpleNeuron::update() {
    Event event = m_waitingList.top();
    m_waitingList.pop();
    m_events.push_back(event);
    return membraneUpdate(event);
}

/* Updates the membrane potential using the newly arrived event.
 * Updates some homeostatic mechanisms such as the refractory period, potential decay and spike rate adaptation.
 * If the membrane potential exceeds the threshold, the neuron spikes.
 */
inline bool SimpleNeuron::membraneUpdate(Event event) {
    m_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / conf.TAU_M);
    m_adaptationPotential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / conf.TAU_SRA);
    m_potential += m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y())
                   - refractoryPotential(event.timestamp() - m_spikingTime)
                   - m_adaptationPotential;
    m_timestampLastEvent = event.timestamp();

    if (m_potential > m_threshold) {
        spike(event.timestamp());
        return true;
    }
    return false;
}

/* Updates the spike timings and spike counters.
 * Also increases the secondary membrane potential use in the spike rate adaptation mechanism.
 */
inline void SimpleNeuron::spike(const long time) {
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

/* Updates the synaptic weights using the STDP learning rule.
 * Only the synapses from which events arrived are updated.
 * Normalizes the weights after the update.
 */
inline void SimpleNeuron::weightUpdate() {
    if (conf.STDP_LEARNING == "excitatory" || conf.STDP_LEARNING == "all") {
        for (Event &event: m_events) {
            m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) +=
                    conf.ETA_LTP * exp(-static_cast<double>(m_spikingTime - event.timestamp()) / conf.TAU_LTP);
            m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) +=
                    conf.ETA_LTD * exp(-static_cast<double>(event.timestamp() - m_lastSpikingTime) / conf.TAU_LTD);

            if (m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) < 0) {
                m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) = 0;
            }
        }
        normalizeWeights();

        //    m_learningDecay = 1 / (1 + exp(m_totalSpike - m_networkConf.DECAY_FACTOR));
    } else if (conf.STDP_LEARNING == "inhibitory" || conf.STDP_LEARNING == "all") {
        for (NeuronEvent &event : m_inhibEvents) {
            m_inhibWeights(event.x(), event.y(), event.z()) += conf.ETA_ILTP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / conf.TAU_LTP);
            m_inhibWeights(event.x(), event.y(), event.z()) += conf.ETA_ILTD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / conf.TAU_LTD);

            if (m_inhibWeights(event.x(), event.y(), event.z()) < 0) {
                m_inhibWeights(event.x(), event.y(), event.z()) = 0;
            }
        }
    }
    m_events.clear();
    m_inhibEvents.clear();
}

/* Weight normalization using tensor calculations.
 */
inline void SimpleNeuron::normalizeWeights() {
    const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& d = m_weights.dimensions();

//    for (long p = 0; p < d[0]; ++p) {
//        for (int c = 0; c < d[1]; ++c) {
//            for (long s = 0; s < d[2]; ++s) {
//                Eigen::array<long, SIMPLEDIM> start = {p, c, s, 0, 0};
//                Eigen::array<long, SIMPLEDIM> size = {1, 1, 1, d[3], d[4]};
//                Eigen::Tensor<double, 0> norm = m_weights.slice(start, size).pow(2).sum().sqrt();
//
//                if (norm(0) != 0) {
//                    m_weights.slice(start, size) = conf.NORM_FACTOR * m_weights.slice(start, size) / norm(0);
//                }
//            }
//        }
//    }

    // weight normalization on the polarity axes
//    for (long p = 0; p < d[0]; ++p) {
//        Eigen::array<long, SIMPLEDIM> start = {p, 0, 0, 0, 0};
//        Eigen::array<long, SIMPLEDIM> size = {1, d[1], d[2], d[3], d[4]};
//        Eigen::Tensor<double, 0> norm = m_weights.slice(start, size).pow(2).sum().sqrt();
//
//        if (norm(0) != 0) {
//            m_weights.slice(start, size) = conf.NORM_FACTOR * m_weights.slice(start, size) / norm(0);
//        }
//    }

    Eigen::Tensor<double, 0> norm = m_weights.pow(2).sum().sqrt();

    if (norm(0) != 0) {
        m_weights = conf.NORM_FACTOR * m_weights / norm(0);
    }
}

void SimpleNeuron::saveWeights(std::string &filePath) {
    auto weightsFile = filePath + std::to_string(m_index);
    Util::saveSimpleTensorToNumpyFile(m_weights, weightsFile);
    weightsFile = filePath + std::to_string(m_index) + "inhib";
    Util::saveComplexTensorToNumpyFile(m_inhibWeights, weightsFile);
}

void SimpleNeuron::loadWeights(std::string &filePath) {
    auto weightsFile = filePath + std::to_string(m_index);
    Util::loadNumpyFileToSimpleTensor(weightsFile, m_weights);
    weightsFile = filePath + std::to_string(m_index) + "inhib";
    Util::loadNumpyFileToComplexTensor(weightsFile, m_inhibWeights);
}

std::vector<long> SimpleNeuron::getWeightsDimension() {
    const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& dimensions = m_weights.dimensions();
    std::vector<long> dim = { dimensions[3], dimensions[4] };
    return dim;
}
