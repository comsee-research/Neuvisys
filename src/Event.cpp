#include "Event.hpp"

Event::Event(long timestamp, int x, int y, bool polarity) : m_timestamp(timestamp), m_x(x), m_y(y), m_polarity(polarity) {}

long Event::timestamp() const {
    return m_timestamp;
}

int Event::x() {
    return m_x;
}

int Event::y() {
    return m_y;
}

bool Event::polarity() {
    return m_polarity;
}