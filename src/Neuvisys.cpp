//
// Created by Thomas on 04/06/2021.
//

#include "src/network/NetworkHandle.hpp"

int main(int argc, char *argv[]) {
    if (argc > 2) {
        std::string networkPath = argv[1];
        std::string eventPath = argv[2];
        size_t nbPass = std::stoi(argv[3]);

        NetworkHandle network(networkPath, eventPath);
        std::vector<Event> events;
        std::cout << "Feeding network... " << std::endl;

        while (network.loadEvents(events, nbPass)) {
            network.feedEvents(events);
        }
        network.save(eventPath, 1);
    } else if (argc > 1) {
        NetworkConfig::createNetwork(argv[1]);
    } else {
        std::cout << "too few arguments, entering debug mode" << std::endl;

        std::string networkPath = "/home/thomas/Desktop/orientation_basis/";
        std::string eventsPath = "/home/thomas/Videos/simulation/full_rotation.npz";

        NetworkHandle network(networkPath, eventsPath);
        std::vector<Event> events;
        std::cout << "Feeding network... " << std::endl;

        while (network.loadEvents(events, 1)) {
            network.feedEvents(events);
        }

        network.save(eventsPath, 1);
    }
}
