#ifndef NEUVISYS_DV_EVENT_HPP
#define NEUVISYS_DV_EVENT_HPP

#include <cstdint>

class Event {
    int64_t m_timestamp;
    uint16_t m_x;
    uint16_t m_y;
    bool m_polarity;
    uint16_t m_camera;
    uint16_t m_synapse;
public:
    Event() = default;
    inline Event(int64_t timestamp, uint16_t x, uint16_t y, bool polarity) : m_timestamp(timestamp), m_x(x), m_y(y), m_polarity(polarity), m_camera(0) { m_synapse = 0; }
    inline Event(int64_t timestamp, uint16_t x, uint16_t y, bool polarity, uint16_t camera) : m_timestamp(timestamp), m_x(x), m_y(y), m_polarity(polarity), m_camera(camera) { m_synapse = 0; }
    inline Event(int64_t timestamp, uint16_t x, uint16_t y, bool polarity, uint16_t camera, uint16_t synapse) : m_timestamp(timestamp), m_x(x), m_y(y), m_polarity(polarity), m_camera(camera), m_synapse(synapse) {}
    [[nodiscard]] inline int64_t timestamp() const {return m_timestamp;}
    [[nodiscard]] inline uint16_t x() const {return m_x;}
    [[nodiscard]] inline uint16_t y() const {return m_y;}
    [[nodiscard]] inline bool polarity() const {return m_polarity;}
    [[nodiscard]] inline uint16_t synapse() const {return m_synapse;}
    [[nodiscard]] inline uint16_t camera() const {return m_camera;}
};

class NeuronEvent {
    int64_t m_timestamp;
    int32_t m_x;
    int32_t m_y;
    int32_t m_z;
public:
    NeuronEvent() = default;
    inline NeuronEvent(int64_t timestamp, int32_t x, int32_t y, int32_t z) : m_timestamp(timestamp), m_x(x), m_y(y), m_z(z) {}
    [[nodiscard]] inline int64_t timestamp() const {return m_timestamp;}
    [[nodiscard]] inline int32_t x() const {return m_x;}
    [[nodiscard]] inline int32_t y() const {return m_y;}
    [[nodiscard]] inline int32_t z() const {return m_z;}
};

#endif //NEUVISYS_DV_EVENT_HPP
