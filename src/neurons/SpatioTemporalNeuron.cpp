#include "SpatioTemporalNeuron.hpp"

SpatioTemporalNeuron::SpatioTemporalNeuron(int x, int y, xt::xarray<double> weights, std::vector<long> delays, double threshold) : OrientedNeuron(x, y, std::move(weights), threshold) {
    m_waitingList = std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>();
    m_delays = std::move(delays);
}

inline void SpatioTemporalNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    int synapse = 0;
    for (auto delay : m_delays) {
        m_waitingList.emplace(timestamp + delay, x, y, polarity, synapse++);
    }
}

bool SpatioTemporalNeuron::update(const long time) {
    while (!m_waitingList.empty() && m_waitingList.top().timestamp() <= time) {
        Event event = m_waitingList.top();
        m_waitingList.pop();
        m_events.push_back(event);

        double decay = m_potential * potentialDecay(event.timestamp() - m_timestampLastEvent);
        double grp = DELTA_RP * refractoryPeriod(time - m_spikingTime);
        m_potential = decay + m_weights(event.polarity(), event.synapse(), event.y(), event.x()) - grp;

        m_timestampLastEvent = event.timestamp();

        if (m_potential > m_threshold) {
            spike(event.timestamp());
        }
    }
}

inline void SpatioTemporalNeuron::spike(const long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    m_spike = true;
    ++m_countSpike;
    ++m_totalSpike;
    m_potential = VRESET;
    learnWeightsSTDP();
    m_events.clear();

    // Tracking
    m_spikeTrain.push_back(time);
}

inline void SpatioTemporalNeuron::learnWeightsSTDP() {
    for (Event &event : m_events) {
        m_weights(event.polarity(), event.synapse(), event.y(), event.x()) += DELTA_VP * exp(- static_cast<double>(m_spikingTime - event.timestamp()) / TAU_LTP);
        m_weights(event.polarity(), event.synapse(), event.y(), event.x()) -= DELTA_VD * exp(- static_cast<double>(event.timestamp() - m_lastSpikingTime) / TAU_LTD);

        if (m_weights(event.polarity(), event.synapse(), event.y(), event.x()) < 0) {
            m_weights(event.polarity(), event.synapse(), event.y(), event.x()) = 0;
        }
    }

    /***** Weights Normalization *****/
    if (m_totalSpike % NORM_THRESHOLD == 0) {
        normalize();
    }
}

inline void SpatioTemporalNeuron::normalize() {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < m_delays.size(); ++j) {
            double norm = xt::linalg::norm(xt::view(m_weights, i, j));
            if (norm != 0) {
                xt::view(m_weights, i, j) = NORM_FACTOR * (xt::view(m_weights, i, j) / norm);
            }
        }
    }
}

double SpatioTemporalNeuron::getWeights(int p, int s, int x, int y) {
    return m_weights(p, s, y, x);
}
