#include "src/SpikingNetwork.hpp"
#include <chrono>
#include <random>
#include <utility>
#include "src/dependencies/json.hpp"
#include "cnpy.h"

using json = nlohmann::json;

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
    for (size_t i = 0; i < 4 * array.shape[0]; i += 4) {
        long timestamp = static_cast<long>(events[i]);
        int x = static_cast<int>(events[i + 1]), y = static_cast<int>(events[i + 2]);
        bool polarity = static_cast<bool>(events[i + 3]);

        spinet.addEvent(timestamp, x, y,polarity);

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

void runSpikingNetwork(cnpy::NpyArray &array) {
    std::string confFile = Conf::CONF_FILE;
    NetworkConfig config = NetworkConfig(confFile);
    std::cout << "Initializing Network " << std::endl;
    SpikingNetwork spinet(config);
    std::map<std::string, cv::Mat> displays;

    main_loop(array, spinet, displays);
}

void alternateSNN() {
    auto array_h = loadEvents("/home/thomas/Vidéos/samples/npy/bars_horizontal.npy");
    auto array_v = loadEvents("/home/thomas/Vidéos/samples/npy/bars_vertical.npy");

    for (int inp = 0; inp < 4; ++inp) {
        for (int ite = 0; ite < 5; ++ite) {
            std::cout << "Pass " << ite+1 << ", Input: " << inp % 2 << std::endl;

            if (inp % 2 == 0) {
                runSpikingNetwork(array_h);
            } else {
                runSpikingNetwork(array_v);
            }
        }
    }
}

void rotateEvents(cnpy::NpyArray &array, double degreeOfRotation) {
    auto *events = array.data<double>();

    int height = 260;
    int width = 346;
    int centerX = (width - 1) / 2;
    int centerY = (height - 1) / 2;
    double cosOfDegree = std::cos(degreeOfRotation * static_cast<double>(M_PI) / static_cast<double>(180.0f));
    double sinOfDegree = std::sin(degreeOfRotation * static_cast<double>(M_PI) / static_cast<double>(180.0f));

    for (size_t i = 0; i < 4 * array.shape[0]; i += 4) {
        int x = static_cast<int>(events[i + 1]), y = static_cast<int>(events[i + 2]);

        if (double(degreeOfRotation) != 0.0) {
            auto dx = static_cast<double>(x - centerX);
            auto dy = static_cast<double>(y - centerY);

            x = static_cast<int16_t>(dx * static_cast<double>(cosOfDegree) - dy * static_cast<double>(sinOfDegree) + centerX);
            y = static_cast<int16_t>(dx * static_cast<double>(sinOfDegree) + dy * static_cast<double>(cosOfDegree) + centerY);

            if (x >= width || x < 0) {
                continue;
            }
            if (y >= height || y < 0) {
                continue;
            }
        }
        events[i + 1] = x;
        events[i + 2] = y;
    }
}

void presentMultipleRotations() {
    auto array = loadEvents("/home/thomas/Vidéos/samples/npy/bars_horizontal.npy");
    std::vector<double> maxResponse;

    for (int degreeOfRotation = -180; degreeOfRotation <= 180; degreeOfRotation += 5) {
        rotateEvents(array, degreeOfRotation);

        std::cout << "Rotation = " << degreeOfRotation << "°" << std::endl;
        runSpikingNetwork(array);

        json state;
        std::ifstream ifs("/home/thomas/neuvisys-dv/configuration/network/weights/complex_cells/0.json");
        if (ifs.is_open()) {
            try {
                ifs >> state;
            } catch (const std::exception& e) {
                std::cerr << "Error opening parameters" << std::endl;
                throw;
            }

            double moy = 0;
            for (const auto &potential : state["potential_train"]) {
                moy += static_cast<double>(potential[0]);
            }
            moy /= static_cast<double>(state["potential_train"].size());
            maxResponse.push_back(moy);
        } else {
        std::cout << "cannot open neuron state file" << std::endl;
        }
        ifs.close();
    }

    int rotation = -180;
    for(const auto &max : maxResponse) {
        std::cout << rotation << "° = " << max << std::endl;
        rotation += 5;
    }
}

void presentRotations(int degreeOfRotation) {
    auto array = loadEvents("/home/thomas/Vidéos/samples/npy/bars_horizontal.npy");
    std::vector<double> maxResponse;

    rotateEvents(array, degreeOfRotation);

    std::cout << "Rotation = " << degreeOfRotation << "°" << std::endl;
    runSpikingNetwork(array);

    json state;
    std::ifstream ifs("/home/thomas/neuvisys-dv/configuration/network/weights/complex_cells/0.json");
    if (ifs.is_open()) {
        try {
            ifs >> state;
        } catch (const std::exception& e) {
            std::cerr << "Error opening parameters" << std::endl;
            throw;
        }

        double moy = 0;
        for (const auto &potential : state["potential_train"]) {
            moy += static_cast<double>(potential[0]);
        }
        moy /= static_cast<double>(state["potential_train"].size());
        maxResponse.push_back(moy);
    } else {
        std::cout << "cannot open neuron state file" << std::endl;
    }
    ifs.close();

    int rotation = -180;
    for(const auto &max : maxResponse) {
        std::cout << rotation << "° = " << max << std::endl;
        rotation += 5;
    }
}

int main(int argc, char *argv[]) {
//    alternateSNN();
//    presentMultipleRotations();
    presentRotations(0);
}
