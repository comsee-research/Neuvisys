//
// Created by thomas on 06/07/22.
//

#include "NetworkHandleTest.hpp"

NetworkHandle* NetworkHandleTest::network = nullptr;
std::string NetworkHandleTest::eventsPath;
std::string NetworkHandleTest::networkPath;

void NetworkHandleTest::SetUpTestSuite() {
    eventsPath = "../../resources/shapes.h5";
    EXPECT_EQ(std::filesystem::exists("../../resources/shapes.h5"), true);
    networkPath = "../../resources/network_test/";

    NetworkConfig::createNetwork("../../resources/network_test");
    if (network == nullptr) {
        network = new NetworkHandle(networkPath, eventsPath);
    }
}

void NetworkHandleTest::TearDownTestSuite() {
    delete network;
    network = nullptr;
    std::filesystem::remove_all(networkPath);
}

TEST_F(NetworkHandleTest, runningNetwork) {
    while (network->loadEvents(events, 1)) {
        network->feedEvents(events);
    }
}

TEST_F(NetworkHandleTest, savingNetwork) {
    network->save(eventsPath, 1);
}