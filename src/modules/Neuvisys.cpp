#include "src/SpikingNetwork.hpp"
#include <chrono>
#include <random>
#include <utility>
#include "src/dependencies/json.hpp"
#include "cnpy.h"

using json = nlohmann::json;

void init_display(NetworkConfig &conf, std::map<std::string, cv::Mat> &displays) {
    displays["frames"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);
    displays["potentials"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
    displays["spikes"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
    displays["weights"] = cv::Mat::zeros(static_cast<int>(conf.Neuron1Height), static_cast<int>(conf.Neuron1Width), CV_8UC3);
    displays["zoom"] = cv::Mat::zeros(static_cast<int>(conf.Neuron1Height), static_cast<int>(conf.Neuron1Width), CV_8UC3);
    displays["potentials2"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
    displays["weights2"] = cv::Mat::zeros(static_cast<int>(conf.Neuron2Height), static_cast<int>(conf.Neuron2Width), CV_8UC3);
    displays["spikes2"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
}

inline void runSpikingNetwork(SpikingNetwork &spinet, Event &event, size_t count, size_t sizeArray) {
    spinet.addEvent(event);

//    if (count % Conf::EVENT_FREQUENCY == 0) {
//        spinet.updateNeurons(event.timestamp());
//    }
//    if (count % Conf::DISPLAY_FREQUENCY == 0) {
//        spinet.updateDisplay(event.timestamp(), displays);
//    }
    if (count % Conf::UPDATE_PARAMETER_FREQUENCY == 0) {
        std::cout << 100 * count / sizeArray << "%" << std::endl;
        spinet.updateNeuronsParameters(event.timestamp());
    }
//        if (count % 100 == 0) {
//            spinet.trackNeuron(timestamp);
//        }
}

void main_loop(SpikingNetwork &spinet, const std::string &array, std::map<std::string, cv::Mat> &displays, size_t nbPass = 1) {
    size_t pass, j, count = 0;

    cnpy::NpyArray timestamps_array = cnpy::npz_load(array, "arr_0");
    cnpy::NpyArray x_array = cnpy::npz_load(array, "arr_1");
    cnpy::NpyArray y_array = cnpy::npz_load(array, "arr_2");
    cnpy::NpyArray polarities_array = cnpy::npz_load(array, "arr_3");
    size_t sizeArray = timestamps_array.shape[0];

    auto *timestamps = timestamps_array.data<long>();
    auto *x = x_array.data<int16_t>();
    auto *y = y_array.data<int16_t>();
    auto *polarities = polarities_array.data<bool>();

    long firstTimestamp = timestamps[0];
    long lastTimestamp = 0;
    auto event = Event();

    for (pass = 0; pass < nbPass; ++pass) {
        for (j = 0; j < sizeArray; ++j) {
            event = Event(timestamps[j] + static_cast<long>(pass) * (lastTimestamp - firstTimestamp), x[j], y[j], polarities[j], 0);
            runSpikingNetwork(spinet, event, count, nbPass * sizeArray);
            ++count;
        }
        std::cout << "Finished iteration: " << pass + 1 << std::endl;
        lastTimestamp = static_cast<long>(timestamps[j-1]);
    }
}

void stereo_loop(SpikingNetwork &spinet, const std::string &leftArray, const std::string &rightArray, std::map<std::string, cv::Mat> &displays, size_t nbPass = 1) {
    size_t pass, count = 0, left, right;

    cnpy::NpyArray l_timestamps_array = cnpy::npz_load(leftArray, "arr_0");
    cnpy::NpyArray l_x_array = cnpy::npz_load(leftArray, "arr_1");
    cnpy::NpyArray l_y_array = cnpy::npz_load(leftArray, "arr_2");
    cnpy::NpyArray l_polarities_array = cnpy::npz_load(leftArray, "arr_3");
    size_t sizeLeftArray = l_timestamps_array.shape[0];

    auto *l_timestamps = l_timestamps_array.data<long>();
    auto *l_x = l_x_array.data<long>();
    auto *l_y = l_y_array.data<long>();
    auto *l_polarities = l_polarities_array.data<bool>();

    cnpy::NpyArray r_timestamps_array = cnpy::npz_load(rightArray, "arr_0");
    cnpy::NpyArray r_x_array = cnpy::npz_load(leftArray, "arr_1");
    cnpy::NpyArray r_y_array = cnpy::npz_load(leftArray, "arr_2");
    cnpy::NpyArray r_polarities_array = cnpy::npz_load(leftArray, "arr_3");
    size_t sizeRightArray = r_timestamps_array.shape[0];

    auto *r_timestamps = r_timestamps_array.data<long>();
    auto *r_x = r_x_array.data<long>();
    auto *r_y = r_y_array.data<long>();
    auto *r_polarities = r_polarities_array.data<bool>();

    long firstLeftTimestamp = l_timestamps[0], firstRightTimestamp = r_timestamps[0], lastLeftTimestamp = 0, lastRightTimestamp = 0;
    auto event = Event();

    for (pass = 0; pass < nbPass; ++pass) {
        left = 0; right = 0;
        while (left < sizeLeftArray && right < sizeRightArray) {
            if (right >= sizeRightArray || l_timestamps[left] <= r_timestamps[right]) {
                event = Event(l_timestamps[left] + static_cast<long>(pass) * (lastLeftTimestamp - firstLeftTimestamp), l_x[left], l_y[left], l_polarities[left], 0);
                ++left;
            } else {
                event = Event(r_timestamps[right] + static_cast<long>(pass) * (lastRightTimestamp - firstRightTimestamp), r_x[right], r_y[right], r_polarities[right], 1);
                ++right;
            }
            runSpikingNetwork(spinet, event, count, nbPass * (sizeLeftArray + sizeRightArray));
            ++count;
        }
        std::cout << "Finished iteration: " << pass + 1 << std::endl;
        lastLeftTimestamp = static_cast<long>(l_timestamps[left-1]);
        lastRightTimestamp = static_cast<long>(r_timestamps[right-1]);
    }
}
//if (right >= sizeRightArray || l_timestamps[left] <= r_timestamps[right]) {
//if (l_x[left] >= 0 && l_y[left] >= 0 && l_x[left] < 346 && l_y[left] < 256) {
//event = Event(l_timestamps[left] + static_cast<long>(pass) * (lastLeftTimestamp - firstLeftTimestamp),
//              l_x[left], l_y[left], l_polarities[left], 0);
//++left;
//} else {
//std::cout << "Wrong left event position " << "x = " << l_x[left] << " y = " << l_y[left] << std::endl;
//++left;
//}
//} else {
//if (r_x[right] >= 0 && r_y[right] >= 0 && r_x[right] < 346 && r_y[right] < 256) {
//event = Event(
//        r_timestamps[right] + static_cast<long>(pass) * (lastRightTimestamp - firstRightTimestamp),
//        r_x[right], r_y[right], r_polarities[right], 1);
//++right;
//} else {
//std::cout << "Wrong right event position " << "x = " << l_x[left] << " y = " << l_y[left] << std::endl;
//++right;
//}
//}
cnpy::NpyArray loadEvents(std::string filePath) {
    std::cout << "Loading Events" << " (file: " << filePath << ")" << std::endl;
    return cnpy::npy_load(std::move(filePath));
}

void multiplePass(std::string networkPath, const std::string &filePath, size_t nbPass) {
    NetworkConfig config = NetworkConfig(std::move(networkPath));
    std::cout << "Initializing Network " << std::endl;
    SpikingNetwork spinet(config);
    std::map<std::string, cv::Mat> displays;
    main_loop(spinet, filePath, displays, nbPass);
}

void stereo(std::string networkPath, const std::string &leftFilePath, const std::string &rightFilePath, size_t nbPass) {
    NetworkConfig config = NetworkConfig(std::move(networkPath));

    if (config.NbCameras == 2) {
        std::cout << "Initializing Network " << std::endl;
        SpikingNetwork spinet(config);
        std::map<std::string, cv::Mat> displays;
        stereo_loop(spinet, leftFilePath, rightFilePath, displays, nbPass);
    } else {
        std::cout << "Incorrect number of cameras for a stereo setup" << std::endl;
    }
}

int main(int argc, char *argv[]) {
    if (argc > 2) {
        if (strcmp(argv[2], "multi-pass") == 0) {
            multiplePass(argv[1], argv[3], static_cast<size_t>(std::stoi(argv[4])));
        }
        else if (strcmp(argv[2], "stereo") == 0) {
            stereo(argv[1], argv[3], argv[4], static_cast<size_t>(std::stoi(argv[5])));
        }
    } else {
        std::cout << "too few arguments, entering debug mode" << std::endl;
//        std::string networkPath = "/home/alphat/neuvisys-dv/configuration/network/configs/network_config.json";
//        std::string left = "/home/alphat/Desktop/l_events.npz";
//        std::string right = "/home/alphat/Desktop/r_events.npz";
//
//        stereo(networkPath, left, right, 1);

        std::string networkPath = "/home/alphat/neuvisys-dv/configuration/NETWORKS/stdps/exp_sym/configs/network_config.json";
        std::string events = "/media/alphat/SSD Games/Thesis/videos/artificial_videos/lines_npz/0.npz";

        multiplePass(networkPath, events, 1);
    }
}
