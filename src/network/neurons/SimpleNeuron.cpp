#include "SimpleNeuron.hpp"

/* Similar to the abstract neuron class.
 * It also takes as input a weight tensor and the number of synapses used when delayed synapses are defined.
 */
SimpleNeuron::SimpleNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, Eigen::Tensor<double, SIMPLEDIM> &weights, size_t nbSynapses) :
        Neuron(index, layer, conf, pos, offset),
        m_events(boost::circular_buffer<Event>(1000)),
        m_topDownInhibitionEvents(boost::circular_buffer<NeuronEvent>(1000)),
        m_lateralInhibitionEvents(boost::circular_buffer<NeuronEvent>(1000)),
        m_weights(weights),
        m_waitingList(std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>()) {
    for (size_t synapse = 0; synapse < nbSynapses; synapse++) {
        m_delays.push_back(static_cast<size_t>(synapse * conf.SYNAPSE_DELAY));
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

void SimpleNeuron::newTopDownInhibitoryEvent(NeuronEvent event) {
    m_topDownInhibitionEvents.push_back(event);
    m_potential -= m_topDownInhibitionWeights.at(event.id());
}

void SimpleNeuron::newLateralInhibitoryEvent(NeuronEvent event) {
    m_lateralInhibitionEvents.push_back(event);
    m_potential -= m_lateralInhibitionWeights.at(event.id());
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
    m_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / m_conf.TAU_M);
    m_adaptationPotential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / m_conf.TAU_SRA);
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
inline void SimpleNeuron::spike(const size_t time) {
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

/* Updates the synaptic weights using the STDP learning rule.
 * Only the synapses from which events arrived are updated.
 * Normalizes the weights after the update.
 */
inline void SimpleNeuron::weightUpdate() {
    if (m_conf.STDP_LEARNING == "excitatory" || m_conf.STDP_LEARNING == "all") {
        for (Event &event: m_events) {
            m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) +=
                    m_decay * m_conf.ETA_LTP * exp(-static_cast<double>(m_spikingTime - event.timestamp()) / m_conf.TAU_LTP);
            m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) +=
                    m_decay * m_conf.ETA_LTD * exp(-static_cast<double>(event.timestamp() - m_lastSpikingTime) / m_conf.TAU_LTD);

            if (m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) < 0) {
                m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) = 0;
            }
        }
        normalizeWeights();
    }
    if (m_conf.STDP_LEARNING == "inhibitory" || m_conf.STDP_LEARNING == "all") {
        for (NeuronEvent &event : m_topDownInhibitionEvents) {
            m_topDownInhibitionWeights.at(event.id()) += m_conf.ETA_ILTP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / m_conf.TAU_LTP);
            m_topDownInhibitionWeights.at(event.id()) += m_conf.ETA_ILTD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / m_conf.TAU_LTD);

            if (m_topDownInhibitionWeights.at(event.id()) < 0) {
                m_topDownInhibitionWeights.at(event.id()) = 0;
            }
        }

        for (NeuronEvent &event : m_lateralInhibitionEvents) {
            m_lateralInhibitionWeights.at(event.id()) += m_conf.ETA_ILTP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / m_conf.TAU_LTP);
            m_lateralInhibitionWeights.at(event.id()) += m_conf.ETA_ILTD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / m_conf.TAU_LTD);

            if (m_lateralInhibitionWeights.at(event.id()) < 0) {
                m_lateralInhibitionWeights.at(event.id()) = 0;
            }
        }
    }
    m_events.clear();
    m_topDownInhibitionEvents.clear();
    m_lateralInhibitionEvents.clear();
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
//                    m_weights.slice(start, size) = m_conf.NORM_FACTOR * m_weights.slice(start, size) / norm(0);
//                }
//            }
//        }
//    }

    // weight normalization on the camera axes
//    for (long c = 0; c < d[1]; ++c) {
//        Eigen::array<long, SIMPLEDIM> start = {0, c, 0, 0, 0};
//        Eigen::array<long, SIMPLEDIM> size = {d[0], 1, d[2], d[3], d[4]};
//        Eigen::Tensor<double, 0> norm = m_weights.slice(start, size).pow(2).sum().sqrt();
//
//        if (norm(0) != 0) {
//            m_weights.slice(start, size) = m_conf.NORM_FACTOR * m_weights.slice(start, size) / norm(0);
//        }
//    }

    Eigen::Tensor<double, 0> norm = m_weights.pow(2).sum().sqrt();

    if (norm(0) != 0) {
        m_weights = m_conf.NORM_FACTOR * m_weights / norm(0);
    }
}

void SimpleNeuron::saveWeights(std::string &filePath) {
    auto arrayName = std::to_string(m_index);
    Util::saveSimpleTensorToNPZ(m_weights, filePath, arrayName);
}

void SimpleNeuron::saveLateralInhibitionWeights(std::string &filePath) {
    auto arrayName = std::to_string(m_index);
    Util::saveWeightsToNPZ(m_lateralInhibitionWeights, filePath, arrayName);
}

void SimpleNeuron::saveTopDownInhibitionWeights(std::string &filePath) {
    auto arrayName = std::to_string(m_index);
    Util::saveWeightsToNPZ(m_topDownInhibitionWeights, filePath, arrayName);
}

void SimpleNeuron::loadWeights(std::string &filePath) {
    if (Util::endsWith(filePath, ".npz")) {
        auto arrayName = std::to_string(m_index);
        Util::loadNumpyFileToSimpleTensor(m_weights, filePath, arrayName);
    } else if (Util::endsWith(filePath, ".npy")) {
        Util::loadNumpyFileToSimpleTensor(m_weights, filePath, filePath);
    }
}

void SimpleNeuron::loadLateralInhibitionWeights(std::string &filePath) {
    if (Util::endsWith(filePath, ".npz")) {
        auto arrayName = std::to_string(m_index);
        Util::loadNPYWeights(m_lateralInhibitionWeights, filePath, arrayName);
    } else if (Util::endsWith(filePath, ".npy")) {
        Util::loadNPYWeights(m_lateralInhibitionWeights, filePath, filePath);
    }
}

void SimpleNeuron::loadTopDownInhibitionWeights(std::string &filePath) {
    if (Util::endsWith(filePath, ".npz")) {
        auto arrayName = std::to_string(m_index);
        Util::loadNPYWeights(m_topDownInhibitionWeights, filePath, arrayName);
    } else if (Util::endsWith(filePath, ".npy")) {
        Util::loadNPYWeights(m_topDownInhibitionWeights, filePath, filePath);
    }
}

std::vector<long> SimpleNeuron::getWeightsDimension() {
    const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& dimensions = m_weights.dimensions();
    std::vector<long> dim = { dimensions[3], dimensions[4] };
    return dim;
}
