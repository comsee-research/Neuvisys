#include "src/SpikingNetwork.hpp"
//#include "src/matplotlibcpp.h"
#include <chrono>
#include <random>
#include "src/dependencies/json.hpp"
//namespace plt = matplotlibcpp;
using json = nlohmann::json;

void main_loop(SpikingNetwork &spinet, std::vector<cv::Mat> &displays) {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int16_t> xs(0, 345);
    std::uniform_int_distribution<int16_t> ys(0, 259);
    std::uniform_int_distribution<int16_t> ps(0, 1);

    long count = 0;
    while (count < 20000000) {
        for (size_t i = 0; i < 10000; ++i) {
            spinet.addEvent(count, xs(mt), ys(mt), ys(mt));
//            displays[0].at<cv::Vec3b>(ys(mt), xs(mt))[2-ys(mt)] = 255;
            ++count;
        }
        spinet.updateNeurons(count);
//        spinet.updateNeuronsParameters();
    }
}

int main() {
    std::string confFile = Conf::CONF_FILE;
    NetworkConfig config = NetworkConfig(confFile);

    SpikingNetwork spinet(config);
    std::vector<cv::Mat> displays;
    displays.push_back(cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3));

    auto start = std::chrono::system_clock::now();
    main_loop(spinet, displays);
    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << std::endl;
}
