//
// Created by alphat on 14/04/2021.
//

#include "SpikingNetwork.hpp"

std::vector<Event> mono(const std::string& events, size_t nbPass) {
    std::vector<Event> eventPacket;
    size_t pass, count;

    cnpy::NpyArray timestamps_array = cnpy::npz_load(events, "arr_0");
    cnpy::NpyArray x_array = cnpy::npz_load(events, "arr_1");
    cnpy::NpyArray y_array = cnpy::npz_load(events, "arr_2");
    cnpy::NpyArray polarities_array = cnpy::npz_load(events, "arr_3");
    size_t sizeLeftArray = timestamps_array.shape[0];

    auto *l_timestamps = timestamps_array.data<long>();
    auto *l_x = x_array.data<int16_t>();
    auto *l_y = y_array.data<int16_t>();
    auto *l_polarities = polarities_array.data<bool>();

    long firstTimestamp = l_timestamps[0];
    long lastTimestamp = static_cast<long>(l_timestamps[sizeLeftArray-1]);
    Event event{};

    for (pass = 0; pass < static_cast<size_t>(nbPass); ++pass) {
        for (count = 0; count < sizeLeftArray; ++count) {
            event = Event(l_timestamps[count] + static_cast<long>(pass) * (lastTimestamp - firstTimestamp), l_x[count], l_y[count], l_polarities[count], 0);
            eventPacket.push_back(event);
        }
    }
    return eventPacket;
}

std::vector<Event> stereo(const std::string& events, size_t nbPass) {
    std::vector<Event> eventPacket;
    size_t pass, left, right;

    cnpy::NpyArray l_timestamps_array = cnpy::npz_load(events, "arr_0");
    cnpy::NpyArray l_x_array = cnpy::npz_load(events, "arr_1");
    cnpy::NpyArray l_y_array = cnpy::npz_load(events, "arr_2");
    cnpy::NpyArray l_polarities_array = cnpy::npz_load(events, "arr_3");
    size_t sizeLeftArray = l_timestamps_array.shape[0];

    auto *l_timestamps = l_timestamps_array.data<long>();
    auto *l_x = l_x_array.data<int16_t>();
    auto *l_y = l_y_array.data<int16_t>();
    auto *l_polarities = l_polarities_array.data<bool>();

    cnpy::NpyArray r_timestamps_array = cnpy::npz_load(events, "arr_4");
    cnpy::NpyArray r_x_array = cnpy::npz_load(events, "arr_5");
    cnpy::NpyArray r_y_array = cnpy::npz_load(events, "arr_6");
    cnpy::NpyArray r_polarities_array = cnpy::npz_load(events, "arr_7");
    size_t sizeRightArray = r_timestamps_array.shape[0];

    auto *r_timestamps = r_timestamps_array.data<long>();
    auto *r_x = r_x_array.data<int16_t>();
    auto *r_y = r_y_array.data<int16_t>();
    auto *r_polarities = r_polarities_array.data<bool>();

    long firstLeftTimestamp = l_timestamps[0], firstRightTimestamp = r_timestamps[0], lastLeftTimestamp = static_cast<long>(l_timestamps[sizeLeftArray-1]), lastRightTimestamp = static_cast<long>(r_timestamps[sizeRightArray-1]);
    long l_t, r_t;
    Event event{};

    for (pass = 0; pass < static_cast<size_t>(nbPass); ++pass) {
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
            eventPacket.push_back(event);
        }
    }
    return eventPacket;
}

void multiplePass(const std::string& networkPath, const std::string& events, size_t nbPass) {
    NetworkConfig config = NetworkConfig(networkPath);
    std::cout << "Initializing Network " << std::endl;
    SpikingNetwork spinet(config);

    auto eventPacket = std::vector<Event>();
    if (spinet.getNetworkConfig().NbCameras == 1) {
        eventPacket = mono(events, nbPass);
    } else if (spinet.getNetworkConfig().NbCameras == 2) {
        eventPacket = stereo(events, nbPass);
    }
    spinet.run(eventPacket);
    std::cout << "Finished" << std::endl;
}