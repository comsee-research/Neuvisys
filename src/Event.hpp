#ifndef NEUVISYS_DV_EVENT_HPP
#define NEUVISYS_DV_EVENT_HPP

#endif //NEUVISYS_DV_EVENT_HPP

class Event {
private:
    long m_timestamp;
    int m_x;
    int m_y;
    bool m_polarity;
public:
    Event();
    Event(long timestamp, int x, int y, bool polarity);
    long timestamp() const;
    int x();
    int y();
    bool polarity();
};
