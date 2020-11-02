#ifndef NEUVISYS_DV_EVENT_HPP
#define NEUVISYS_DV_EVENT_HPP

#include <cstdint>

class Event {
    long m_timestamp;
    long m_x;
    long m_y;
    bool m_polarity;
    long m_synapse;
public:
    Event() = default;
    inline Event(long timestamp, long x, long y, bool polarity) : m_timestamp(timestamp), m_x(x), m_y(y), m_polarity(polarity) {m_synapse = 0;}
    inline Event(long timestamp, long x, long y, bool polarity, long synapse) : m_timestamp(timestamp), m_x(x), m_y(y), m_polarity(polarity), m_synapse(synapse) {}
    [[nodiscard]] inline long timestamp() const {return m_timestamp;}
    [[nodiscard]] inline long x() const {return m_x;}
    [[nodiscard]] inline long y() const {return m_y;}
    [[nodiscard]] inline bool polarity() const {return m_polarity;}
    [[nodiscard]] inline long synapse() const {return m_synapse;}
};

class NeuronEvent {
    long m_timestamp;
    long m_x;
    long m_y;
    long m_z;
public:
    NeuronEvent() = default;
    inline NeuronEvent(long timestamp, long x, long y, long z) : m_timestamp(timestamp), m_x(x), m_y(y), m_z(z) {}
    [[nodiscard]] inline long timestamp() const {return m_timestamp;}
    [[nodiscard]] inline long x() const {return m_x;}
    [[nodiscard]] inline long y() const {return m_y;}
    [[nodiscard]] inline long z() const {return m_z;}
};

#endif //NEUVISYS_DV_EVENT_HPP
