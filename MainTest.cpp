#include "src/SpikingNetwork.hpp"
#include <chrono>

int main(int argc, char* argv[]) {
    SpikingNetwork spinet;
    std::vector<cv::Mat> displays;

    for (int i = 0; i < ADJACENT_NEURONS; ++i) {
        displays.emplace_back(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1));
    }

    long count = 0;
    while (count < 1000000) {
        for (size_t i = 0; i < 3000; i++) {
            spinet.addEvent(count, 0, 0, true);
            count++;
        }
        spinet.updateNeuronsInformation(count, displays);
    }
}
