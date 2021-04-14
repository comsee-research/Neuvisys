#include "src/network/NetworkHandle.hpp"

int main(int argc, char *argv[]) {
    if (argc > 2) {
        if (strcmp(argv[2], "multi-pass") == 0) {
            auto neuvisys = NetworkHandle(argv[1], argv[3], static_cast<size_t>(std::stoi(argv[4])));
            neuvisys.multiplePass();
        }
    } else {
        std::cout << "too few arguments, entering debug mode" << std::endl;

        std::string networkPath = "/home/alphat/Desktop/Networks/network_0/configs/network_config.json";
        std::string events = "/home/alphat/Desktop/shapes.npz";

        auto neuvisys = NetworkHandle(networkPath, events, 1);
        neuvisys.multiplePass();
    }
}
