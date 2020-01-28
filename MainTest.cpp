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
    GnuplotPipe gp;
    gp.sendLine("set title \"hey\"");
    std::string plot = "plot[0:5][0:5] '-' lc rgb 'blue' with lines";
    plot += "\n 1 2";
    plot += "\n 2.5 2.5";
    plot += "\n 3 4";
    gp.sendLine(plot);

    for (size_t i = 0; i < NUMBER_DISPLAY; ++i) {
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
