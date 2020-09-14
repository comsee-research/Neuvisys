#include "src/SpikingNetwork.hpp"
#include <chrono>
#include <random>
#include <utility>
#include "src/dependencies/json.hpp"
#include "cnpy.h"

//#include "src/matplotlibcpp.h"
//namespace plt = matplotlibcpp;

void init_display(NetworkConfig &conf, std::map<std::string, cv::Mat> &displays) {
    displays["frames"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);
    displays["potentials"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
    displays["spikes"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
    displays["weights"] = cv::Mat::zeros(conf.Neuron1Height, conf.Neuron1Width, CV_8UC3);
    displays["zoom"] = cv::Mat::zeros(conf.Neuron1Height, conf.Neuron1Width, CV_8UC3);
    displays["potentials2"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
    displays["weights2"] = cv::Mat::zeros(conf.Neuron2Height, conf.Neuron2Width, CV_8UC3);
    displays["spikes2"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
}

void main_loop(cnpy::NpyArray &array, SpikingNetwork &spinet, std::map<std::string, cv::Mat> &displays) {
    int count = 0;
    auto *events = array.data<double>();
    for (size_t i = 0; i < 4*array.shape[0]; i += 4) {
        spinet.addEvent(events[i], events[i+1], events[i+2], events[i+3]);

        if (count % 1000 == 0) {
            spinet.updateNeurons(events[i]);
        }
//        if (count % 30000 == 0) {
//            spinet.updateDisplay(event.timestamp(), displays);
//        }
        if (count % 1000000 == 0) {
            std::cout << 100 * static_cast<size_t>(count) / array.shape[0] << "%" << std::endl;
            spinet.updateNeuronsParameters(events[i]);
        }
        ++count;
    }
}

void spikingNetwork(std::string filePath) {
    std::cout << "Initializing Network, " << "Event file: " << filePath << std::endl;
    std::string confFile = Conf::CONF_FILE;
    NetworkConfig config = NetworkConfig(confFile);
    SpikingNetwork spinet(config);
    std::map<std::string, cv::Mat> displays;
    init_display(config, displays);

    std::cout << "Loading Events" << std::endl;
    cnpy::NpyArray array = cnpy::npy_load(std::move(filePath));

    std::cout << "Launching training" << std::endl;
    auto start = std::chrono::system_clock::now();
    main_loop(array, spinet, displays);
    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << std::endl;
}

int main(int argc, char *argv[]) {
    if (argc > 1) {
        std::string filePath(argv[1]);
        spikingNetwork(filePath);
    } else {
        std::cerr << "Too few arguments" << std::endl;
    }
}
