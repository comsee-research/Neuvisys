//
// Created by Thomas on 14/04/2021.
//

#ifndef NEUVISYS_DV_EVENT_HPP
#define NEUVISYS_DV_EVENT_HPP

#include <cstdint>

class Event {
    uint64_t m_timestamp;
    uint16_t m_x;
    uint16_t m_y;
    bool m_polarity;
    uint16_t m_camera;
    uint16_t m_synapse;

public:
    Event() = default;

    Event(uint64_t timestamp, uint16_t x, uint16_t y, bool polarity, uint16_t camera=0, uint16_t synapse=0) : m_timestamp(timestamp), m_x(x),
                                                                                                                 m_y(y), m_polarity(polarity),
                                                                                                                 m_camera(camera),
                                                                                                                 m_synapse(synapse) {}

    [[nodiscard]] uint64_t timestamp() const { return m_timestamp; }

    [[nodiscard]] uint16_t x() const { return m_x; }

    [[nodiscard]] uint16_t y() const { return m_y; }

    [[nodiscard]] bool polarity() const { return m_polarity; }

    [[nodiscard]] uint16_t synapse() const { return m_synapse; }

    [[nodiscard]] uint16_t camera() const { return m_camera; }

    bool operator<(const Event &event) const {
        return m_timestamp < event.m_timestamp;
    }

    Event &operator=(Event event) {
        m_x = event.x();
        m_y = event.y();
        m_polarity = event.polarity();
        m_timestamp = event.timestamp();
        m_camera = event.camera();
    }
};

class NeuronEvent {
    uint64_t m_timestamp{};
    uint32_t m_x{};
    uint32_t m_y{};
    uint32_t m_z{};
    uint32_t m_id{};
    uint32_t m_layer{};

public:
    NeuronEvent() = default;

    NeuronEvent(uint64_t timestamp, uint32_t id) : m_timestamp(timestamp), m_id(id) {}

    NeuronEvent(uint64_t timestamp, uint32_t x, uint32_t y, uint32_t z, uint32_t layer=0) : m_timestamp(timestamp), m_x(x), m_y(y), m_z(z), m_layer(layer) {}

    [[nodiscard]] uint64_t timestamp() const { return m_timestamp; }

    [[nodiscard]] uint32_t x() const { return m_x; }

    [[nodiscard]] uint32_t y() const { return m_y; }

    [[nodiscard]] uint32_t z() const { return m_z; }

    [[nodiscard]] uint32_t id() const { return m_id; }

    [[nodiscard]] uint32_t layer() const { return m_layer; }
};

#endif //NEUVISYS_DV_EVENT_HPP
