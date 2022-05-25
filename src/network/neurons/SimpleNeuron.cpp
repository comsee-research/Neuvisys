//
// Created by Thomas on 14/04/2021.
//

#include "SimpleNeuron.hpp"

/**
 * Similar to the abstract neuron class.
 * It also takes as input a weight tensor and the number of synapses used when delayed synapses are defined.
 * @param index
 * @param layer
 * @param conf
 * @param pos
 * @param offset
 * @param weights
 * @param nbSynapses
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

/**
 * Updates neuron internal state after the arrival of an event
 * Checks first if there is some synaptic delays defined in the network.
 * @param event
 * @return
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

/**
 *
 * @param event
 */
void SimpleNeuron::newTopDownInhibitoryEvent(NeuronEvent event) {
    m_topDownInhibitionEvents.push_back(event);
    potentialDecay(event.timestamp());
    m_potential -= m_topDownInhibitionWeights.at(event.id());
    m_timestampLastEvent = event.timestamp();
}

/**
 *
 * @param event
 */
void SimpleNeuron::newLateralInhibitoryEvent(NeuronEvent event) {
    m_lateralInhibitionEvents.push_back(event);
    potentialDecay(event.timestamp());
    m_potential -= m_lateralInhibitionWeights.at(event.id());
    m_timestampLastEvent = event.timestamp();
}

/**
 *
 * @return
 */
bool SimpleNeuron::update() {
    Event event = m_waitingList.top();
    m_waitingList.pop();
    m_events.push_back(event);
    return membraneUpdate(event);
}

/**
 * Updates the membrane potential using the newly arrived event.
 * Updates some homeostatic mechanisms such as the refractory period, potential decay and spike rate adaptation.
 * If the membrane potential exceeds the threshold, the neuron spikes.
 * @param event
 * @return
 */
inline bool SimpleNeuron::membraneUpdate(Event event) {
    potentialDecay(event.timestamp());
    adaptationPotentialDecay(event.timestamp());
    m_potential += m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y())
                   - refractoryPotential(event.timestamp())
                   - m_adaptationPotential;
    m_timestampLastEvent = event.timestamp();

    if (m_potential > m_threshold) {
        spike(event.timestamp());
        return true;
    }
    return false;
}

/**
 * Updates the spike timings and spike counters.
 * Also increases the secondary membrane potential use in the spike rate adaptation mechanism.
 * @param time
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

/**
 * Updates the synaptic weights using the STDP learning rule.
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
        Util::normalizeSimpleTensor(m_weights, m_conf.NORM_FACTOR);
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

/**
 *
 * @param filePath
 */
void SimpleNeuron::saveWeights(std::string &filePath) {
    auto weightsFile = filePath + std::to_string(m_index);
    Util::saveSimpleTensorToNumpyFile(m_weights, weightsFile);
//    auto arrayName = std::to_string(m_index);
//    Util::saveSimpleTensorToNumpyFile(m_weights, filePath, arrayName);
}

/**
 *
 * @param filePath
 */
void SimpleNeuron::saveLateralInhibitionWeights(std::string &filePath) {
    auto weightsFile = filePath + std::to_string(m_index) + "li";
    Util::saveWeightsToNumpyFile(m_lateralInhibitionWeights, weightsFile);
//    auto arrayName = std::to_string(m_index);
//    Util::saveWeightsToNumpyFile(m_lateralInhibitionWeights, filePath, arrayName);
}

void SimpleNeuron::saveTopDownInhibitionWeights(std::string &filePath) {
    auto weightsFile = filePath + std::to_string(m_index) + "tdi";
    Util::saveWeightsToNumpyFile(m_topDownInhibitionWeights, weightsFile);
//    auto arrayName = std::to_string(m_index);
//    Util::saveWeightsToNumpyFile(m_topDownInhibitionWeights, filePath, arrayName);
}

/**
 *
 * @param filePath
 */
void SimpleNeuron::loadWeights(std::string &filePath) {
    auto numpyFile = filePath + std::to_string(m_index) + ".npy";
    Util::loadNumpyFileToSimpleTensor(m_weights, numpyFile);
}

/**
 *
 * @param arrayNPZ
 */
void SimpleNeuron::loadWeights(cnpy::npz_t &arrayNPZ) {
    auto arrayName = std::to_string(m_index);
    Util::loadNumpyFileToSimpleTensor(m_weights, arrayNPZ, arrayName);
}

/**
 *
 * @param arrayNPZ
 */
void SimpleNeuron::loadLateralInhibitionWeights(cnpy::npz_t &arrayNPZ) {
    auto arrayName = std::to_string(m_index);
    Util::loadNumpyFileToWeights(m_lateralInhibitionWeights, arrayNPZ, arrayName);
}

/**
 *
 * @param filePath
 */
void SimpleNeuron::loadLateralInhibitionWeights(std::string &filePath) {
    auto numpyFile = filePath + std::to_string(m_index) + "li.npy";
    Util::loadNumpyFileToWeights(m_lateralInhibitionWeights, numpyFile);
}

/**
 *
 * @param arrayNPZ
 */
void SimpleNeuron::loadTopDownInhibitionWeights(cnpy::npz_t &arrayNPZ) {
    auto arrayName = std::to_string(m_index);
    Util::loadNumpyFileToWeights(m_topDownInhibitionWeights, arrayNPZ, arrayName);
}

/**
 *
 * @param filePath
 */
void SimpleNeuron::loadTopDownInhibitionWeights(std::string &filePath) {
    auto numpyFile = filePath + std::to_string(m_index) + "tdi.npy";
    Util::loadNumpyFileToWeights(m_topDownInhibitionWeights, numpyFile);
}

/**
 *
 * @return
 */
std::vector<long> SimpleNeuron::getWeightsDimension() {
    const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& dimensions = m_weights.dimensions();
    std::vector<long> dim = { dimensions[3], dimensions[4] };
    return dim;
}
