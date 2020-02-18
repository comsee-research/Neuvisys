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
    inline long timestamp() const {return m_timestamp;}
    inline int x() {return m_x;}
    inline int y() {return m_y;}
    inline bool polarity() {return m_polarity;}
    inline int synapse() {return m_synapse;}
};

#endif //NEUVISYS_DV_EVENT_HPP
