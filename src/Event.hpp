#ifndef NEUVISYS_DV_EVENT_HPP
#define NEUVISYS_DV_EVENT_HPP

#include <cstdint>

class Event {
    long m_timestamp;
    int m_x;
    int m_y;
    bool m_polarity;
    int m_synapse;
public:
    Event() = default;
    inline Event(long timestamp, int x, int y, bool polarity) : m_timestamp(timestamp), m_x(x), m_y(y), m_polarity(polarity) {m_synapse = 0;}
    inline Event(long timestamp, int x, int y, bool polarity, int synapse) : m_timestamp(timestamp), m_x(x), m_y(y), m_polarity(polarity), m_synapse(synapse) {}
    [[nodiscard]] inline long timestamp() const {return m_timestamp;}
    [[nodiscard]] inline int x() const {return m_x;}
    [[nodiscard]] inline int y() const {return m_y;}
    [[nodiscard]] inline bool polarity() const {return m_polarity;}
    [[nodiscard]] inline int synapse() const {return m_synapse;}
};

class NeuronEvent {
    long m_timestamp;
    int m_x;
    int m_y;
    int m_z;
public:
    NeuronEvent() = default;
    inline NeuronEvent(long timestamp, int x, int y, int z) : m_timestamp(timestamp), m_x(x), m_y(y), m_z(z) {}
    [[nodiscard]] inline long timestamp() const {return m_timestamp;}
    [[nodiscard]] inline int x() const {return m_x;}
    [[nodiscard]] inline int y() const {return m_y;}
    [[nodiscard]] inline int z() const {return m_z;}
};

#endif //NEUVISYS_DV_EVENT_HPP
