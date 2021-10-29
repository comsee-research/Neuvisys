#include "src/network/NetworkHandle.hpp"

int main(int argc, char *argv[]) {
    if (argc > 2) {
        NetworkHandle network(argv[1]);
        network.multiplePass(argv[2], static_cast<size_t>(std::stoi(argv[3])));
    } else if (argc > 1) {
        NetworkConfig::createNetwork(argv[1]);
    } else {
        std::cout << "too few arguments, entering debug mode" << std::endl;

        std::string networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
        std::string events = "/home/thomas/Desktop/shapes.npz";

        NetworkHandle network(networkPath);
        network.multiplePass(events, 1);
    }
}
