#ifndef NEUVISYS_DV_EVENT_HPP
#define NEUVISYS_DV_EVENT_HPP

#include <cstdint>

class Event {
    long m_timestamp;
    size_t m_x;
    size_t m_y;
    bool m_polarity;
    size_t m_synapse;
public:
    Event() = default;
    inline Event(long timestamp, size_t x, size_t y, bool polarity) : m_timestamp(timestamp), m_x(x), m_y(y), m_polarity(polarity) {m_synapse = 0;}
    inline Event(long timestamp, size_t x, size_t y, bool polarity, size_t synapse) : m_timestamp(timestamp), m_x(x), m_y(y), m_polarity(polarity), m_synapse(synapse) {}
    [[nodiscard]] inline long timestamp() const {return m_timestamp;}
    [[nodiscard]] inline size_t x() const {return m_x;}
    [[nodiscard]] inline size_t y() const {return m_y;}
    [[nodiscard]] inline bool polarity() const {return m_polarity;}
    [[nodiscard]] inline size_t synapse() const {return m_synapse;}
};

class NeuronEvent {
    long m_timestamp;
    size_t m_x;
    size_t m_y;
    size_t m_z;
public:
    NeuronEvent() = default;
    inline NeuronEvent(long timestamp, size_t x, size_t y, size_t z) : m_timestamp(timestamp), m_x(x), m_y(y), m_z(z) {}
    [[nodiscard]] inline long timestamp() const {return m_timestamp;}
    [[nodiscard]] inline size_t x() const {return m_x;}
    [[nodiscard]] inline size_t y() const {return m_y;}
    [[nodiscard]] inline size_t z() const {return m_z;}
};

#endif //NEUVISYS_DV_EVENT_HPP
