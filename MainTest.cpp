#include "src/SpikingNetwork.hpp"
#include <chrono>

int main(int argc, char* argv[]) {
    SpikingNetwork spinet;
    std::vector<cv::Mat> displays;

    long count = 0;
    while (count < 1000000) {
        for (size_t i = 0; i < 100; i++) {
            spinet.addEvent(count, 0, 0, true);
            count++;
        }
        spinet.updateNeuronsInformation(count, displays);
    }
}
