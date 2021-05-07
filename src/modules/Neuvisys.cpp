#include "src/network/NetworkHandle.hpp"

int main(int argc, char *argv[]) {
    if (argc > 2) {
        multiplePass(argv[1], argv[2], static_cast<size_t>(std::stoi(argv[3])));
    } else {
        std::cout << "too few arguments, entering debug mode" << std::endl;

        std::string networkPath = "/home/alphat/neuvisys-dv/configuration/network/configs/network_config.json";
        std::string events = "/home/alphat/Desktop/shapes.npz";

        multiplePass(networkPath, events, 1);
    }
}
