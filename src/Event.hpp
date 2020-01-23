#ifndef NEUVISYS_DV_EVENT_HPP
#define NEUVISYS_DV_EVENT_HPP

class Event {
    long m_timestamp;
    int m_x;
    int m_y;
    bool m_polarity;
public:
    Event() = default;
    Event(long timestamp, int x, int y, bool polarity);
    long timestamp() const;
    int x();
    int y();
    bool polarity();
};

#endif //NEUVISYS_DV_EVENT_HPP
