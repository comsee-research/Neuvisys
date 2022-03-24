#include "src/network/NetworkHandle.hpp"

int main(int argc, char *argv[]) {
    if (argc > 2) {
        NetworkHandle network(argv[1], 0);
        network.feedEvents(network.loadEvents(argv[2], static_cast<size_t>(std::stoi(argv[3]))));
        network.save(argv[2], static_cast<size_t>(std::stoi(argv[3])));
    } else if (argc > 1) {
        NetworkConfig::createNetwork(argv[1]);
    } else {
        std::cout << "too few arguments, entering debug mode" << std::endl;

        std::string networkPath = "/home/thomas/Desktop/Experiment/network_0/configs/network_config.json";
        std::string events = "/home/thomas/Videos/lines/sim_vh.npz";

        NetworkHandle network(networkPath, 0);
        network.feedEvents(network.loadEvents(events, 1));
        network.save(events, 1);
    }
}
