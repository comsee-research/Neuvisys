#include "src/SpikingNetwork.hpp"
#include <chrono>
#include <random>
#include "src/dependencies/json.hpp"
//#include "src/matplotlibcpp.h"
//namespace plt = matplotlibcpp;
using json = nlohmann::json;

std::vector<Event> load_aedat(std::string &filePath) {
    json events;

    std::ifstream ifs(filePath);
    if (ifs.is_open()) {
        ifs >> events;
    }
    ifs.close();

    std::vector<Event> vec_events;
    for (auto event : events) {
        vec_events.emplace_back(event[0], event[1], event[2], event[3]);
    }
    return vec_events;
}

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

void main_loop(std::vector<Event> &events, SpikingNetwork &spinet, std::map<std::string, cv::Mat> &displays) {
    int count = 0;
    for (Event event : events) {
        spinet.addEvent(event.timestamp(), event.x(), event.y(), event.polarity());
        displays["frames"].at<cv::Vec3b>(event.y(), event.x())[2-event.y()] = 255;
        ++count;

        if (count % 1000 == 0) {
            spinet.updateNeurons(event.timestamp());
        }

        if (count % 30000 == 0) {
            spinet.updateDisplay(event.timestamp(), displays);
        }

        if (count % 100000 == 0) {
            spinet.updateNeuronsParameters();
        }
    }
}

int main() {
    std::string confFile = Conf::CONF_FILE;
    NetworkConfig config = NetworkConfig(confFile);
    SpikingNetwork spinet(config);
    std::map<std::string, cv::Mat> displays;
    init_display(config, displays);

    if (config.SAVE_DATA) {
        spinet.loadWeights();
    }

    std::string filePath("/home/thomas/Bureau/test");
    auto events = load_aedat(filePath);

    auto start = std::chrono::system_clock::now();
    main_loop(events, spinet, displays);
    auto end = std::chrono::system_clock::now();

    if (config.SAVE_DATA) {
        spinet.saveWeights();
    }

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << std::endl;
}
