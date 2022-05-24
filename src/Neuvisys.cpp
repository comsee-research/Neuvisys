#include "src/network/NetworkHandle.hpp"
#include "src/network/SurroundSuppression.hpp"

int main(int argc, char *argv[]) {
    if (argc > 2) {
        std::string networkPath = argv[1];
        std::string eventsPath = argv[2];
        NetworkHandle network(networkPath, eventsPath);
        int nbCount = atoi(argv[3]);
        std::vector<Event> events;
        std::cout << "argv[3] = " << nbCount << std::endl;
        std::cout << "Feeding network... " << std::endl;

        while (network.loadEvents(events, nbCount)) {
            network.feedEvents(events);
        }

        network.save(eventsPath, nbCount);
        
    } else if (argc > 1) {
        NetworkConfig::createNetwork(argv[1]);
    } else {
        std::cout << "too few arguments, entering debug mode" << std::endl;

        std::string networkPath = "/home/comsee/Internship_Antony/neuvisys/neuvisys-analysis/configuration/other_dataset_training/lateral_topdown/shared/20/vertical_diff_speeds_8/";

        std::string path_Events = "/home/comsee/Internship_Antony/neuvisys/Events/new_bars/events/";
        std::vector<std::string> vectorOfPaths;
        for (const auto & frame : std::filesystem::directory_iterator{path_Events}) 
        {
            vectorOfPaths.emplace_back(frame.path().string());
        //    std::cout << frame.path().string() << std::endl;
        }

        NetworkHandle network(networkPath, vectorOfPaths[0]);
        SurroundSuppression surround(networkPath,vectorOfPaths,network);
        std::string typeOfTraining = "all";
    //    surround.Training(typeOfTraining,2,3);
        surround.launchTrainingNeurons();
    }
}
