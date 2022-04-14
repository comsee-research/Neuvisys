#include "src/network/NetworkHandle.hpp"

int main(int argc, char *argv[]) {
    if (argc > 2) {
        NetworkHandle network(argv[1], 0);
//        network.feedEvents(network.loadEvents(argv[2], static_cast<size_t>(std::stoi(argv[3]))));
//        network.save(argv[2], static_cast<size_t>(std::stoi(argv[3])));
    } else if (argc > 1) {
        NetworkConfig::createNetwork(argv[1]);
    } else {
        std::cout << "too few arguments, entering debug mode" << std::endl;

        std::string networkPath = "/home/thomas/neuvisys-dv/configuration/network/";
        std::string eventsPath = "/home/thomas/Videos/DSEC/interlaken_00_c.h5";

        NetworkHandle network(networkPath, eventsPath);
        std::vector<Event> events;
        std::cout << "Feeding network: " << events.size() << " events..." << std::endl;

        while (network.loadEvents(events, 1)) {
            network.feedEvents(events);
        }

        network.save(eventsPath, 1);
    }
}
