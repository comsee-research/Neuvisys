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

    long count = 0;
    while (count < 2000000) {
        for (size_t i = 0; i < 20000; ++i) {
//            spinet.addEvent(count, xs(mt), ys(mt), true);
            spinet.addEvent(count, 0, 0, true);
//            spinet.addEvent(count, 0, 10, true);
            ++count;
        }
    }
}


int main() {
    std::string confFile = CONF_FILE;
    Config::loadConfiguration(confFile);

    std::string configurationFile = CONF_FILES_LOCATION;
    Config::loadNetworkLayout(configurationFile);
    Config::loadNeuronsParameters(configurationFile);

    SpikingNetwork spinet;
    std::vector<cv::Mat> displays;

    auto start = std::chrono::system_clock::now();
    main_loop(spinet, displays);
    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << std::endl;

    std::cout << spinet.getNeuron(0).getWeights(0, 0, 0, 0) << std::endl;
    std::cout << spinet.getNeuron(4).getWeights(0, 0, 0, 0) << std::endl;
}
