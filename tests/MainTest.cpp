#include "src/network/SpikingNetwork.hpp"
#include <chrono>
#include <random>
#include <utility>
#include "src/dependencies/json.hpp"

//#include "src/matplotlibcpp.h"
//namespace plt = matplotlibcpp;

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

void main_loop(cnpy::NpyArray &array, SpikingNetwork &spinet, std::map<std::string, cv::Mat> &displays) {
    size_t count = 0;
    auto *events = array.data<double>();
    for (size_t i = 0; i < 4 * array.shape[0]; i += 4) {
        long timestamp = static_cast<long>(events[i]);
        auto x = static_cast<size_t>(events[i + 1]), y = static_cast<size_t>(events[i + 2]);
        bool polarity = static_cast<bool>(events[i + 3]);

//        spinet.addEvent(timestamp, x, y, polarity);

        if (count % 1000 == 0) {
            spinet.updateNeurons(timestamp);
        }
//        if (count % 30000 == 0) {
//            spinet.updateDisplay(timestamp, displays);
//        }
        if (count % 1000000 == 0) {
            std::cout << 100 * static_cast<size_t>(count) / array.shape[0] << "%" << std::endl;
            spinet.updateNeuronsParameters(timestamp);
        }
        if (count % 100 == 0) {
            spinet.trackNeuron(timestamp);
        }
        ++count;
    }
}

cnpy::NpyArray loadEvents(std::string filePath) {
    std::cout << "Loading Events" << " (file: " << filePath << ")" << std::endl;
    return cnpy::npy_load(std::move(filePath));
}

void testSpikingNetwork(std::string &filePath) {
    auto array = loadEvents(filePath);

//    std::string confFile = Conf::CONF_FILE;
//    NetworkConfig config = NetworkConfig(confFile);

//    std::cout << "Initializing Network " << std::endl;
//    SpikingNetwork spinet(config);
//    std::map<std::string, cv::Mat> displays;
//    init_display(config, displays);

//    std::cout << "Launching training" << std::endl;
//    main_loop(array, spinet, displays);
}

int main(int argc, char *argv[]) {
    std::string filePath = "/home/alphat/Videos/shape_hovering.npy";
    testSpikingNetwork(filePath);
}
