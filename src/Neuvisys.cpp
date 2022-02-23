#include "src/network/NetworkHandle.hpp"

int main(int argc, char *argv[]) {
    if (argc > 2) {
        NetworkHandle network(argv[1], 0);
        network.multiplePass(argv[2], static_cast<size_t>(std::stoi(argv[3])));
    } else if (argc > 1) {
        NetworkConfig::createNetwork(argv[1]);
    } else {
        std::cout << "too few arguments, entering debug mode" << std::endl;

        std::string networkPath = "/home/thomas/Desktop/Experiment/network_0/configs/network_config.json";
        std::string events = "/home/thomas/Videos/shapes.npz";

        NetworkHandle network(networkPath, 0);
        network.multiplePass(events, 10);
    }
}
