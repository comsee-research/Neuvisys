#include "src/SpikingNetwork.hpp"
#include <chrono>
#include <random>

void main_loop(SpikingNetwork &spinet, std::vector<cv::Mat> &displays) {
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int16_t> xs(0, 345);
    std::uniform_int_distribution<int16_t> ys(0, 259);

    long count = 0;
    while (count < 1000) {
        for (size_t i = 0; i < 20000; ++i) {
            spinet.addEvent(count, xs(mt), ys(mt), true);
        }
        ++count;
    }
}

int main(int argc, char* argv[]) {
    SpikingNetwork spinet;
    std::vector<cv::Mat> displays;

    for (size_t i = 0; i < ADJACENT_NEURONS; ++i) {
        displays.emplace_back(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1));
    }

    auto start = std::chrono::system_clock::now();

//    std::thread t1(main_loop, spinet);
//    t1.join();
    main_loop(spinet, displays);

    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "elapsed time: " << elapsed_seconds.count() << std::endl;
}
