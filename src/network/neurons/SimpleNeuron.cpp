#include "SimpleNeuron.hpp"

SimpleNeuron::SimpleNeuron(size_t index, size_t layer, NeuronConfig &conf, Position pos, Position offset, Eigen::Tensor<double, SIMPLEDIM> &weights, size_t nbSynapses) :
    Neuron(index, layer, conf, pos, offset),
    m_events(boost::circular_buffer<Event>(1000)),
    m_weights(weights),
    m_waitingList(std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>()) {
    for (size_t synapse = 0; synapse < nbSynapses; synapse++) {
        m_delays.push_back(static_cast<long>(synapse * conf.SYNAPSE_DELAY));
    }
}

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

bool SimpleNeuron::update() {
    Event event = m_waitingList.top();
    m_waitingList.pop();
    m_events.push_back(event);
    return membraneUpdate(event);
}

inline bool SimpleNeuron::membraneUpdate(Event event) {
    m_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / conf.TAU_M);
    m_adaptation_potential *= exp(- static_cast<double>(event.timestamp() - m_timestampLastEvent) / conf.TAU_SRA);
//    potentialDecay(event.timestamp() - m_timestampLastEvent);
//    adaptationPotentialDecay(event.timestamp() - m_timestampLastEvent);
    m_potential += m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y())
                   - refractoryPotential(event.timestamp() - m_spikingTime)
                   - m_adaptation_potential;
    m_timestampLastEvent = event.timestamp();

    if (m_potential > m_threshold) {
        spike(event.timestamp());
        return true;
    }
    return false;
}

inline void SimpleNeuron::spike(const long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_countSpike;
    ++m_totalSpike;
    m_potential = conf.VRESET;

    if (conf.TRACKING == "partial") {
//        m_trackingSpikeTrain.push_back(time);
    }
}

inline void SimpleNeuron::weightUpdate() {
    if (conf.STDP_LEARNING) {
        for (Event &event : m_events) {
            m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) += m_learningDecay * conf.ETA_LTP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / conf.TAU_LTP);
            m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) += m_learningDecay * conf.ETA_LTD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / conf.TAU_LTD);

            if (m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) < 0) {
                m_weights(event.polarity(), event.camera(), event.synapse(), event.x(), event.y()) = 0;
            }
        }

        normalizeWeights();
        //    m_learningDecay = 1 / (1 + exp(m_totalSpike - m_conf.DECAY_FACTOR));
    }
    m_events.clear();
}

inline void SimpleNeuron::normalizeWeights() {
    const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& d = m_weights.dimensions();

    for (long p = 0; p < d[0]; ++p) {
        for (int c = 0; c < d[1]; ++c) {
            for (long s = 0; s < d[2]; ++s) {
                Eigen::array<long, SIMPLEDIM> start = {p, c, s, 0, 0};
                Eigen::array<long, SIMPLEDIM> size = {1, 1, 1, d[3], d[4]};
                Eigen::Tensor<double, 0> norm = m_weights.slice(start, size).pow(2).sum().sqrt();

                if (norm(0) != 0) {
                    m_weights.slice(start, size) = conf.NORM_FACTOR * m_weights.slice(start, size) / norm(0);
                }
            }
        }
    }
}

void SimpleNeuron::saveWeights(std::string &saveFile) {
    Util::saveSimpleTensorToNumpyFile(m_weights, saveFile);
}

void SimpleNeuron::loadWeights(std::string &filePath) {
    Util::loadNumpyFileToSimpleTensor(filePath, m_weights);
}

double SimpleNeuron::getWeights(long p, long c, long s, long x, long y) {
    return m_weights(p, c, s, x, y);
}

std::vector<long> SimpleNeuron::getWeightsDimension() {
    const Eigen::Tensor<double, SIMPLEDIM>::Dimensions& dimensions = m_weights.dimensions();
    std::vector<long> dim = { dimensions[3], dimensions[4] };
    return dim;
}
