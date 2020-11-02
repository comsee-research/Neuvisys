#include "SimpleNeuron.hpp"

SimpleNeuron::SimpleNeuron(size_t index, NeuronConfig &conf, Luts &luts, Position pos, Position offset, Eigen::Tensor<double, 4> &weights, size_t nbSynapses) :
    Neuron(index, conf, luts, pos, offset),
    m_events(boost::circular_buffer<Event>(1000)),
    m_weights(weights),
    m_waitingList(std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>()) {
    for (size_t synapse = 0; synapse < nbSynapses; synapse++) {
        m_delays.push_back(static_cast<long>(synapse * conf.SYNAPSE_DELAY));
    }
}

inline void SimpleNeuron::newEvent(const long timestamp, const long x, const long y, const bool polarity) {
    if ((m_delays.size() == 1) && (m_delays[0] == 0)) {
        membraneUpdate(timestamp, x, y, polarity, 0);
        m_events.push_back(Event(timestamp, x, y, polarity, 0));
    } else {
        size_t synapse = 0;
        for (auto delay : m_delays) {
            m_waitingList.emplace(timestamp + delay, x, y, polarity, synapse++);
        }
    }
}

void SimpleNeuron::update(const long time) {
    while (!m_waitingList.empty() && m_waitingList.top().timestamp() <= time) {
        Event event = m_waitingList.top();
        m_waitingList.pop();
        m_events.push_back(event);

        membraneUpdate(event.timestamp(), event.x(), event.y(), event.polarity(), event.synapse());
    }
}

inline void SimpleNeuron::membraneUpdate(const long timestamp, const long x, const long y, const bool polarity, const long synapse) {
    if (timestamp - m_timestampLastEvent < 1000000) {
        m_potential *= m_luts.lutM[static_cast<size_t>(timestamp - m_timestampLastEvent)];
    } else {
        m_potential = 0;
    }
    if (timestamp - m_timestampLastEvent < 1000000) {
        m_adaptation_potential *= m_luts.lutM[static_cast<size_t>(timestamp - m_timestampLastEvent)];
    } else {
        m_adaptation_potential = 0;
    }
//    potentialDecay(timestamp - m_timestampLastEvent);
//    adaptationPotentialDecay(timestamp - m_timestampLastEvent);
    m_potential += m_weights(polarity, synapse, x, y)
                   - refractoryPotential(timestamp - m_spikingTime)
                   - m_adaptation_potential;
    m_timestampLastEvent = timestamp;

    if (m_potential > m_threshold) {
        spike(timestamp);
    }
}

inline void SimpleNeuron::spike(const long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_countSpike;
    ++m_totalSpike;
    m_potential = conf.VRESET;

    spikeRateAdaptation();
    if (conf.STDP_LEARNING) {
        updateSTDP();
    }
    m_events.clear();

    if (conf.TRACKING) {
        m_spikeTrain.push_back(time);
    }
}

inline void SimpleNeuron::updateSTDP() {
    for (Event &event : m_events) {
        m_weights(event.polarity(), event.synapse(), event.x(), event.y()) += m_learningDecay * conf.DELTA_VP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / conf.TAU_LTP);
        m_weights(event.polarity(), event.synapse(), event.x(), event.y()) -= m_learningDecay * conf.DELTA_VD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / conf.TAU_LTD);

        if (m_weights(event.polarity(), event.synapse(), event.x(), event.y()) < 0) {
            m_weights(event.polarity(), event.synapse(), event.x(), event.y()) = 0;
        }
    }

    normalizeWeights();
//    m_learningDecay = 1 / (1 + exp(m_totalSpike - conf.DECAY_FACTOR));
}

inline void SimpleNeuron::normalizeWeights() {
    const Eigen::Tensor<double, 4>::Dimensions& d = m_weights.dimensions();

    for (long p = 0; p < d[0]; ++p) {
        for (long s = 0; s < d[1]; ++s) {
            Eigen::array<long, 4> start = {p, s, 0, 0};
            Eigen::array<long, 4> size = {1, 1, d[2], d[3]};
            Eigen::Tensor<double, 0> norm = m_weights.slice(start, size).pow(2).sum().sqrt();

            if (norm(0) != 0) {
                m_weights.slice(start, size) = conf.NORM_FACTOR * m_weights.slice(start, size) / norm(0);
            }
        }
    }
}

void SimpleNeuron::saveWeights(std::string &saveFile) {
    Util::save4DTensorToNumpyFile(m_weights, saveFile);
}

void SimpleNeuron::loadWeights(std::string &filePath) {
    Util::loadNumpyFileTo4DTensor(filePath, m_weights);
}

double SimpleNeuron::getWeights(long p, long s, long x, long y) {
    return m_weights(p, s, x, y);
}
