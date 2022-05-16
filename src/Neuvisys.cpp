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

    //    std::string networkPath = "/home/comsee/Internship_Antony/neuvisys/neuvisys-cpp/configuration/network_dir/";
    //    std::string eventsPath = "/home/comsee/Internship_Antony/datasets/shapes.npz";
        std::string networkPath = "/home/comsee/Internship_Antony/neuvisys/neuvisys-analysis/configuration/other_dataset_training/lateral/vertical_diff_speeds/";
    //    std::string eventsPath0 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_0.npz";
/*        std::string eventsPath0 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_18t.npz";

        std::string eventsPath2 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_2.npz";
        std::string eventsPath3 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_3.npz";
//        std::string eventsPath4 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_4.npz";
        std::string eventsPath5 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_5.npz";
        std::string eventsPath6 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_6.npz";
        std::string eventsPath7 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_7.npz";
        std::string eventsPath8 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_8.npz";
        std::string eventsPath9 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_9.npz";
        std::string eventsPath10 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_10.npz";
        std::string eventsPath11 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_11.npz";
        std::string eventsPath12 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_12.npz";
        std::string eventsPath13 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_12.npz";
        std::string eventsPath14a = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_13.npz";
        std::string eventsPath14b = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_14b.npz";
        std::string eventsPath15 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_15.npz";
        std::string eventsPath16 = "/home/comsee/Internship_Antony/neuvisys/Events/different_bars_speeds/events/test_16.npz";*/

        std::string path_Events = "/home/comsee/Internship_Antony/neuvisys/Events/new_bars/events/";
        std::vector<std::string> vectorOfPaths;
        for (const auto & frame : std::filesystem::directory_iterator{path_Events}) 
        {
            vectorOfPaths.emplace_back(frame.path().string());
        //    std::cout << frame.path().string() << std::endl;
        }
/*        vectorOfPaths.push_back(eventsPath0);
        vectorOfPaths.push_back(eventsPath2);
        vectorOfPaths.push_back(eventsPath3);
        vectorOfPaths.push_back(eventsPath5);
        vectorOfPaths.push_back(eventsPath6);
        vectorOfPaths.push_back(eventsPath7);
        vectorOfPaths.push_back(eventsPath8);
        vectorOfPaths.push_back(eventsPath9);
        vectorOfPaths.push_back(eventsPath10);
        vectorOfPaths.push_back(eventsPath11);
        vectorOfPaths.push_back(eventsPath12);
        vectorOfPaths.push_back(eventsPath13);
        vectorOfPaths.push_back(eventsPath14a);
        vectorOfPaths.push_back(eventsPath14b);
        vectorOfPaths.push_back(eventsPath15);
        vectorOfPaths.push_back(eventsPath16);*/

        NetworkHandle network(networkPath, vectorOfPaths[0]);
        SurroundSuppression surround(networkPath,vectorOfPaths,network);
        std::string typeOfTraining = "all";
    //    surround.Training(typeOfTraining,1,2);
        surround.launchTrainingNeurons(0);

    /*    NetworkHandle network0(networkPath, eventsPath0);
        std::vector<Event> events;
        std::cout << "event packet number 0 " << " ; Feeding network... " << std::endl;
        int nb=10;
        while (network0.loadEvents(events, nb)) {
            network0.feedEvents(events);
        }
        network0.save(eventsPath0, nb);
        network0.setEventPath(eventsPath2);
        std::cout << "It worked?!" << std::endl;*/
/*        NetworkHandle network2(networkPath, eventsPath2);
        std::vector<Event> events2;
        std::cout << "event packet number 2 " << " ; Feeding network... " << std::endl;
        while (network2.loadEvents(events2, nb)) {
            network2.feedEvents(events2);
        }
        network2.save(eventsPath2, nb);*/

/*        NetworkHandle network3(networkPath, eventsPath3);
        std::vector<Event> events3;
        std::cout << "event packet number 3 " << " ; Feeding network... " << std::endl;
        while (network3.loadEvents(events3, nb)) {
            network3.feedEvents(events3);
        }
        network3.save(eventsPath3, nb);*/

/*        NetworkHandle network4(networkPath, eventsPath4);
        std::vector<Event> events4;
        std::cout << "event packet number 4 " << " ; Feeding network... " << std::endl;
        while (network4.loadEvents(events4, nb)) {
            network4.feedEvents(events4);
        }
        network4.save(eventsPath4, nb);*/

/*        NetworkHandle network5(networkPath, eventsPath5);
        std::vector<Event> events5;
        std::cout << "event packet number 5 " << " ; Feeding network... " << std::endl;
        while (network5.loadEvents(events5, nb)) {
            network5.feedEvents(events5);
        }
        network5.save(eventsPath5, nb);*/

/*        NetworkHandle network6(networkPath, eventsPath6);
        std::vector<Event> events6;
        std::cout << "event packet number 6 " << " ; Feeding network... " << std::endl;
        while (network6.loadEvents(events6, nb)) {
            network6.feedEvents(events6);
        }
        network6.save(eventsPath6, nb);*/

/*        NetworkHandle network7(networkPath, eventsPath7);
        std::vector<Event> events7;
        std::cout << "event packet number 7 " << " ; Feeding network... " << std::endl;
        while (network7.loadEvents(events7, nb)) {
            network7.feedEvents(events7);
        }
        network7.save(eventsPath7, nb);*/

/*        NetworkHandle network8(networkPath, eventsPath8);
        std::vector<Event> events8;
        std::cout << "event packet number 8 " << " ; Feeding network... " << std::endl;
        while (network8.loadEvents(events8, nb)) {
            network8.feedEvents(events8);
        }
        network8.save(eventsPath8, nb);*/

/*        NetworkHandle network9(networkPath, eventsPath9);
        std::vector<Event> events9;
        std::cout << "event packet number 9 " << " ; Feeding network... " << std::endl;
        while (network9.loadEvents(events9, nb)) {
            network9.feedEvents(events9);
        }
        network9.save(eventsPath9, nb);*/

/*        NetworkHandle network10(networkPath, eventsPath10);
        std::vector<Event> events10;
        std::cout << "event packet number 10 " << " ; Feeding network... " << std::endl;
        while (network10.loadEvents(events10, nb)) {
            network10.feedEvents(events10);
        }
        network10.save(eventsPath10, nb);*/

/*        NetworkHandle network11(networkPath, eventsPath11);
        std::vector<Event> events11;
        std::cout << "event packet number 11 " << " ; Feeding network... " << std::endl;
        while (network11.loadEvents(events11, nb)) {
            network11.feedEvents(events11);
        }
        network11.save(eventsPath11, nb);*/

/*        NetworkHandle network12(networkPath, eventsPath12);
        std::vector<Event> events12;
        std::cout << "event packet number 12 " << " ; Feeding network... " << std::endl;
        while (network12.loadEvents(events12, nb)) {
            network12.feedEvents(events12);
        }
        network12.save(eventsPath12, nb);*/

/*        NetworkHandle network13(networkPath, eventsPath13);
        std::vector<Event> events13;
        std::cout << "event packet number 13 " << " ; Feeding network... " << std::endl;
        while (network13.loadEvents(events13, nb)) {
            network13.feedEvents(events13);
        }
        network13.save(eventsPath13, nb);*/

/*        NetworkHandle network14a(networkPath, eventsPath14a);
        std::vector<Event> events14a;
        std::cout << "event packet number 14a " << " ; Feeding network... " << std::endl;
        while (network14a.loadEvents(events14a, nb)) {
            network14a.feedEvents(events14a);
        }
        network14a.save(eventsPath14a, nb);*/

/*        NetworkHandle network14b(networkPath, eventsPath14b);
        std::vector<Event> events14b;
        std::cout << "event packet number 14b " << " ; Feeding network... " << std::endl;
        while (network14b.loadEvents(events14b, nb)) {
            network14b.feedEvents(events14b);
        }
        network14b.save(eventsPath14b, nb);*/

/*        NetworkHandle network15(networkPath, eventsPath15);
        std::vector<Event> events15;
        std::cout << "event packet number 15 " << " ; Feeding network... " << std::endl;
        while (network15.loadEvents(events15, nb)) {
            network15.feedEvents(events15);
        }
        network15.save(eventsPath15, nb);*/

/*        NetworkHandle network16(networkPath, eventsPath16);
        std::vector<Event> events16;
        std::cout << "event packet number 16 " << " ; Feeding network... " << std::endl;
        while (network16.loadEvents(events16, nb)) {
            network16.feedEvents(events16);
        }
        network16.save(eventsPath16, nb);*/
    }
}
