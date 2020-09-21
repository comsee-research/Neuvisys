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
        spinet.addEvent(static_cast<long>(events[i]), static_cast<int>(events[i + 1]), static_cast<int>(events[i + 2]),
                        static_cast<bool>(events[i + 3]));

        if (count % 1000 == 0) {
            spinet.updateNeurons(static_cast<long>(events[i]));
        }
//        if (count % 30000 == 0) {
//            spinet.updateDisplay(event.timestamp(), displays);
//        }
        if (count % 1000000 == 0) {
            std::cout << 100 * static_cast<size_t>(count) / array.shape[0] << "%" << std::endl;
            spinet.updateNeuronsParameters(static_cast<long>(events[i]));
        }
        if (count % 100 == 0) {
            spinet.trackNeuron(static_cast<long>(events[i]));
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

    std::string confFile = Conf::CONF_FILE;
    NetworkConfig config = NetworkConfig(confFile);

    std::cout << "Initializing Network " << std::endl;
    SpikingNetwork spinet(config);
    std::map<std::string, cv::Mat> displays;
    init_display(config, displays);

    std::cout << "Launching training" << std::endl;
    main_loop(array, spinet, displays);
}

void tailoredSpikingNetwork() {
    auto array_h = loadEvents("/home/thomas/Vidéos/samples/npy/bars_horizontal.npy");
    auto array_v = loadEvents("/home/thomas/Vidéos/samples/npy/bars_vertical.npy");
//    auto array = loadEvents("/home/thomas/Vidéos/samples/npy/shape_slow_hovering.npy");

    std::string confFile = Conf::CONF_FILE;
    NetworkConfig config = NetworkConfig(confFile);

    for (int inp = 0; inp < 4; ++inp) {
        for (int ite = 0; ite < 5; ++ite) {
            std::cout << "Pass " << ite+1 << ", Input: " << inp % 2 << std::endl;

            std::cout << "Initializing Network " << std::endl;
            SpikingNetwork spinet(config);
            std::map<std::string, cv::Mat> displays;
            //init_display(config, displays);

            std::cout << "Launching training" << std::endl;
            if (inp % 2 == 0) {
                main_loop(array_h, spinet, displays);
            } else {
                main_loop(array_v, spinet, displays);
            }
        }
    }
}

int main(int argc, char *argv[]) {
    if (argc > 1) {
        std::string filePath(argv[1]);
        testSpikingNetwork(filePath);
    } else {
        tailoredSpikingNetwork();
    }
}
