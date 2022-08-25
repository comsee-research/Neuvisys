//
// Created by Thomas on 04/06/2021.
//

#include <network/NetworkHandle.hpp>
#include <network/SurroundSuppression.hpp>
#include <network/config/DefaultConfig.hpp>

int main(int argc, char *argv[]) {
    if (argc > 2) {
        std::string networkPath = argv[1];
        std::string eventsPath = argv[2];
        NetworkHandle network(networkPath, eventsPath);
        size_t nbCount = std::atoi(argv[3]);
        std::vector<Event> events;
        std::cout << "argv[3] = " << nbCount << std::endl;
        std::cout << "Feeding network... " << std::endl;

        while (network.loadEvents(events, nbCount)) {
            network.feedEvents(events);
        }

        network.save(eventsPath, nbCount);
    } else if (argc > 1) {
        NetworkConfig::createNetwork(argv[1], PredefinedConfigurations::twoLayerOnePatchWeightSharingCenteredConfig);
    } else {
    /*    std::cout << "too few arguments, entering debug mode" << std::endl;
        std::string networkPath = "/home/thomas/neuvisys-dv/src/resources/network_test/";
        std::string eventsPath = "/home/thomas/neuvisys-dv/src/resources/shapes.h5";

        NetworkConfig::createNetwork(networkPath, PredefinedConfigurations::fourLayerRLOnePatchCenteredConfig);

        WeightMatrix::setSeed(15);
        NetworkHandle network(networkPath, eventsPath);
        Events events;
        std::cout << "Feeding network... " << std::endl;
        size_t nbPass = 1;

        while (network.loadEvents(events, nbPass)) {
            network.feedEvents(events);
        }

        network.save(eventsPath, nbPass);*/

        std::cout << "too few arguments, entering debug mode" << std::endl;
    //    std::string networkPath = "/home/comsee/Internship_Antony/neuvisys/neuvisys-analysis/configuration/other_dataset_training/lateral_topdown/shared/rotated/new_rot/new_dataset/rotated_grey1_retryhigh/";
        std::string networkPath = "/home/comsee/Internship_Antony/neuvisys/neuvisys-analysis/configuration/other_dataset_training/lateral_topdown/shared/vertical/";
        std::string path_Events = "/home/comsee/Internship_Antony/neuvisys/Events/rotated_new_bars8/events/";
        std::vector<std::string> vectorOfPaths;
        for (const auto & frame : std::filesystem::directory_iterator{path_Events}) {
            vectorOfPaths.emplace_back(frame.path().string());
        }
        NetworkHandle network(networkPath, vectorOfPaths[0]);
        SurroundSuppression surround(networkPath,vectorOfPaths,network);
        std::string typeOfTraining = "all";
    //    surround.train(typeOfTraining,1,3);
    //    surround.shuffleInhibition(2);
        surround.evaluateResponsesOnStimuli();
    //    surround.evaluateResponsesOnStimuli("/media/comsee/My Passport/test_dataset/");
    //    surround.evaluateResponsesOnStimuli("/home/comsee/Internship_Antony/neuvisys/Events/rotated_new_bars_test2/events/");
    //    surround.evaluateResponsesOnStimuli("/home/comsee/Internship_Antony/neuvisys/Events/nonsynthetic_dataset/");
    }
}
