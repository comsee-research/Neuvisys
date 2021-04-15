//
// Created by alphat on 14/04/2021.
//

#include "NetworkHandle.hpp"

NetworkHandle::NetworkHandle(std::string networkPath, std::string events, size_t nbPass) {
    m_networkPath = std::move(networkPath);
    m_events = std::move(events);
    m_nbPass = nbPass;
    m_iterations = 0;

    m_frameTime = std::chrono::high_resolution_clock::now();
    m_iterations = 0;
}

void NetworkHandle::multiplePass() {
    NetworkConfig config = NetworkConfig(m_networkPath);
    std::cout << "Initializing Network " << std::endl;
    SpikingNetwork spinet(config);
    main_loop(spinet);
    std::cout << "Finished" << std::endl;
}

void NetworkHandle::main_loop(SpikingNetwork &spinet) {
    size_t pass, left, right;

    cnpy::NpyArray l_timestamps_array = cnpy::npz_load(m_events, "arr_0");
    cnpy::NpyArray l_x_array = cnpy::npz_load(m_events, "arr_1");
    cnpy::NpyArray l_y_array = cnpy::npz_load(m_events, "arr_2");
    cnpy::NpyArray l_polarities_array = cnpy::npz_load(m_events, "arr_3");
    size_t sizeLeftArray = l_timestamps_array.shape[0];

    auto *l_timestamps = l_timestamps_array.data<long>();
    auto *l_x = l_x_array.data<int16_t>();
    auto *l_y = l_y_array.data<int16_t>();
    auto *l_polarities = l_polarities_array.data<bool>();

    if (spinet.getNetworkConfig().NbCameras == 1) {
        long firstTimestamp = l_timestamps[0];
        long lastTimestamp = static_cast<long>(l_timestamps[sizeLeftArray-1]);
        auto event = Event();

        for (pass = 0; pass < static_cast<size_t>(m_nbPass); ++pass) {
            for (left = 0; left < sizeLeftArray; ++left) {
                event = Event(l_timestamps[left] + static_cast<long>(pass) * (lastTimestamp - firstTimestamp), l_x[left], l_y[left], l_polarities[left], 0);
                runSpikingNetwork(spinet, event, m_nbPass * sizeLeftArray);
            }
            std::cout << "Finished iteration: " << pass + 1 << std::endl;
        }
    } else if (spinet.getNetworkConfig().NbCameras == 2) {
        cnpy::NpyArray r_timestamps_array = cnpy::npz_load(m_events, "arr_4");
        cnpy::NpyArray r_x_array = cnpy::npz_load(m_events, "arr_5");
        cnpy::NpyArray r_y_array = cnpy::npz_load(m_events, "arr_6");
        cnpy::NpyArray r_polarities_array = cnpy::npz_load(m_events, "arr_7");
        size_t sizeRightArray = r_timestamps_array.shape[0];

        auto *r_timestamps = r_timestamps_array.data<long>();
        auto *r_x = r_x_array.data<int16_t>();
        auto *r_y = r_y_array.data<int16_t>();
        auto *r_polarities = r_polarities_array.data<bool>();

        long firstLeftTimestamp = l_timestamps[0], firstRightTimestamp = r_timestamps[0], lastLeftTimestamp = static_cast<long>(l_timestamps[sizeLeftArray-1]), lastRightTimestamp = static_cast<long>(r_timestamps[sizeRightArray-1]);
        long l_t = 0, r_t = 0;
        auto event = Event();

        for (pass = 0; pass < static_cast<size_t>(m_nbPass); ++pass) {
            left = 0; right = 0;
            while (left < sizeLeftArray && right < sizeRightArray) {
                l_t = l_timestamps[left] + static_cast<long>(pass) * (lastLeftTimestamp - firstLeftTimestamp);
                r_t = r_timestamps[right] + static_cast<long>(pass) * (lastRightTimestamp - firstRightTimestamp);
                if (right >= sizeRightArray || l_t <= r_t) {
                    event = Event(l_t / 1000, l_x[left], l_y[left], l_polarities[left], 0);
                    ++left;
                } else if (left >= sizeLeftArray || l_t > r_t) {
                    event = Event(r_t / 1000, r_x[right], r_y[right], r_polarities[right], 1);
                    ++right;
                }
                runSpikingNetwork(spinet, event, m_nbPass * (sizeLeftArray + sizeRightArray));
            }
            std::cout << "Finished iteration: " << pass + 1 << std::endl;
        }
    }
}

inline void NetworkHandle::runSpikingNetwork(SpikingNetwork &spinet, Event &event, size_t sizeArray) {
    spinet.addEvent(event);

//    if (count % Conf::EVENT_FREQUENCY == 0) {
//        spinet.updateNeurons(event.timestamp());
//    }

    std::chrono::duration<double> frameElapsed = std::chrono::high_resolution_clock::now() - m_frameTime;
    if (1000000 * frameElapsed.count() > static_cast<double>(m_precisionEvent)) {
        m_frameTime = std::chrono::high_resolution_clock::now();
    }

    std::chrono::duration<double> trackingElapsed = std::chrono::high_resolution_clock::now() - m_trackingTime;
    if (1000000 * trackingElapsed.count() > static_cast<double>(m_precisionPotential)) {
        m_trackingTime = std::chrono::high_resolution_clock::now();
    }

    if (static_cast<size_t>(m_iterations) % Conf::UPDATE_PARAMETER_FREQUENCY == 0) {
        spinet.updateNeuronsParameters(event.timestamp());
        std::cout << static_cast<size_t>(100 * m_iterations) / sizeArray << "%" << std::endl;
    }
    ++m_iterations;
}