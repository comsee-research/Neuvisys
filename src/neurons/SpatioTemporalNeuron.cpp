#include "SpatioTemporalNeuron.hpp"

SpatioTemporalNeuron::SpatioTemporalNeuron(int x, int y, xt::xarray<double> weights, std::vector<long> delays, double threshold) : OrientedNeuron(x, y, std::move(weights), threshold) {
    m_waitingList = std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>();
    m_delays = std::move(delays);
}

inline bool SpatioTemporalNeuron::newEvent(const long timestamp, const int x, const int y, const bool polarity) {
    if (timestamp > m_inhibitionTime + TAU_INHIB) {
        int count = 0;
        for (auto delay : m_delays) {
            m_waitingList.emplace(timestamp + 1000 * delay, x, y, polarity, count++);
        }
    }
    return false;
}

bool SpatioTemporalNeuron::update(const long time) {
    while (!m_waitingList.empty() && m_waitingList.top().timestamp() <= time) {
        Event event = m_waitingList.top();
        m_waitingList.pop();

        m_events.push_back(event);

        long dt_event = event.timestamp() - m_timestampLastEvent;
        m_potential = potentialDecay(dt_event);
        m_timestampLastEvent = event.timestamp();

        m_potential += m_weights(event.polarity(), event.synapse(), event.y(), event.x());

        if (m_potential > m_threshold) {
            spike(event.timestamp());
            return true;
        }
    }
    return false;
}

inline void SpatioTemporalNeuron::spike(const long time) {
    m_lastSpikingTime = m_spikingTime;
    m_spikingTime = time;
    learnWeightsSTDP();
    m_potential = VRESET;
    m_spike = true;
    m_waitingList = std::priority_queue<Event, std::vector<Event>, CompareEventsTimestamp>();
    m_events.clear();
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
    ++m_countSpike;
    if (m_countSpike % NORM_THRESHOLD == 0) {
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
