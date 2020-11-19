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

void main_loop(SpikingNetwork &spinet, cnpy::NpyArray &array, std::map<std::string, cv::Mat> &displays, size_t nbPass = 1) {
    size_t i, j, count = 0;
    auto *events = array.data<double>();
    long firstTimestamp = static_cast<long>(events[0]);
    long lastTimestamp = 0;

    for (i = 0; i < nbPass; ++i) {
        for (j = 0; j < 4 * array.shape[0]; j += 4) {
            auto event = Event(static_cast<long>(events[j]) + static_cast<long>(i) * (lastTimestamp - firstTimestamp),
                               static_cast<int16_t>(events[j+1]), static_cast<int16_t>(events[j+2]),
                               static_cast<bool>(events[j+3]), 0);
            spinet.addEvent(event);

            if (count % 1000 == 0) {
                spinet.updateNeurons(event.timestamp());
            }
//        if (count % 30000 == 0) {
//            spinet.updateDisplay(timestamp, displays);
//        }
            if (count % 1000000 == 0) {
                std::cout << 100 * count / (nbPass * array.shape[0]) << "%" << std::endl;
                spinet.updateNeuronsParameters(event.timestamp());
            }
//        if (count % 100 == 0) {
//            spinet.trackNeuron(timestamp);
//        }
            ++count;
        }
        std::cout << "Finished iteration: " << i+1 << std::endl;
        lastTimestamp = static_cast<long>(events[j-4]);
    }
}

cnpy::NpyArray loadEvents(std::string filePath) {
    std::cout << "Loading Events" << " (file: " << filePath << ")" << std::endl;
    return cnpy::npy_load(std::move(filePath));
}

void rotateEvents(cnpy::NpyArray &array, double degreeOfRotation) {
    auto *events = array.data<double>();

    int height = 260;
    int width = 346;
    int centerX = (width - 1) / 2;
    int centerY = (height - 1) / 2;
    double cosOfDegree = std::cos(degreeOfRotation * static_cast<double>(M_PI) / static_cast<double>(180.0f));
    double sinOfDegree = std::sin(degreeOfRotation * static_cast<double>(M_PI) / static_cast<double>(180.0f));

    for (size_t i = 0; i < 4 * array.shape[0]; i += 4) {
        int x = static_cast<int>(events[i + 1]), y = static_cast<int>(events[i + 2]);

        if (double(degreeOfRotation) != 0.0) {
            auto dx = static_cast<double>(x - centerX);
            auto dy = static_cast<double>(y - centerY);

            x = static_cast<int16_t>(dx * static_cast<double>(cosOfDegree) - dy * static_cast<double>(sinOfDegree) + centerX);
            y = static_cast<int16_t>(dx * static_cast<double>(sinOfDegree) + dy * static_cast<double>(cosOfDegree) + centerY);

            if (x >= width || x < 0) {
                continue;
            }
            if (y >= height || y < 0) {
                continue;
            }
        }
        events[i + 1] = x;
        events[i + 2] = y;
    }
}

void presentRotation(std::string networkPath, std::string filePath, double degreeOfRotation) {
    auto array = loadEvents(std::move(filePath));
    std::cout << "Rotation " << degreeOfRotation << std::endl;
    rotateEvents(array, degreeOfRotation);

    NetworkConfig config = NetworkConfig(networkPath);
    std::cout << "Initializing Network " << std::endl;
    SpikingNetwork spinet(config);
    std::map<std::string, cv::Mat> displays;
    main_loop(spinet, array, displays);
}

void multiplePass(std::string networkPath, std::string filePath, size_t nbPass) {
    auto array = loadEvents(std::move(filePath));

    NetworkConfig config = NetworkConfig(networkPath);
    std::cout << "Initializing Network " << std::endl;
    SpikingNetwork spinet(config);
    std::map<std::string, cv::Mat> displays;
    main_loop(spinet, array, displays, nbPass);
}

void stereo(std::string networkPath, std::string leftFilePath, std::string rightFilePath, size_t nbPass) {
    auto leftArray = loadEvents(std::move(leftFilePath));
    auto rightArray = loadEvents(std::move(rightFilePath));

    NetworkConfig config = NetworkConfig(networkPath);
    std::cout << "Initializing Network " << std::endl;
    SpikingNetwork spinet(config);
    std::map<std::string, cv::Mat> displays;
    main_loop(spinet, leftArray, displays);
}

int main(int argc, char *argv[]) {
    if (argc > 2) {
        if (strcmp(argv[2], "rotation") == 0) {
            presentRotation(argv[1], argv[3], std::stod(argv[4]));
        }
        else if (strcmp(argv[2], "multi-pass") == 0) {
            multiplePass(argv[1], argv[3], static_cast<size_t>(std::stoi(argv[4])));
        }
        else if (strcmp(argv[2], "stereo") == 0) {
            stereo(argv[1], argv[3], argv[4], static_cast<size_t>(std::stoi(argv[5])));
        }
    } else {
        std::cout << "too few arguments" << std::endl;
    }
}
