//
// Created by thomas on 06/07/22.
//

#include "NetworkHandleTest.hpp"

NetworkHandle* NetworkHandleTest::network = nullptr;
std::string NetworkHandleTest::eventsPath;
std::string NetworkHandleTest::networkPath;
std::string NetworkHandleTest::networkPath2;
std::string NetworkHandleTest::networkPath3;

void NetworkHandleTest::SetUpTestSuite() {
    eventsPath = "../../data/events/shapes.h5";
    EXPECT_EQ(std::filesystem::exists("../../data/events/shapes.h5"), true);
    networkPath = "../../data/networks/network_test/";
    networkPath2 = "../../data/networks/network_rl_test/";
    networkPath3 = "../../data/networks/network_nws_test/";

    NetworkConfig::createNetwork("../../data/networks/network_test", PredefinedConfigurations::twoLayerOnePatchWeightSharingCenteredConfig);
    NetworkConfig::createNetwork("../../data/networks/network_rl_test", PredefinedConfigurations::fourLayerRLOnePatchCenteredConfig);
    NetworkConfig::createNetwork("../../data/networks/network_nws_test", PredefinedConfigurations::oneLayerOnePatchNoWeightSharingConfig);
    if (network == nullptr) {
        WeightMatrix::setSeed(1486546);
        WeightMap::setSeed(461846);
        network = new NetworkHandle(networkPath, eventsPath);
    }
}

void NetworkHandleTest::TearDownTestSuite() {
    delete network;
    network = nullptr;
    std::filesystem::remove_all(networkPath);
    std::filesystem::remove_all(networkPath2);
    std::filesystem::remove_all(networkPath2);
}

TEST_F(NetworkHandleTest, runningNetwork) {
    while (network->loadEvents(events, 1)) {
        network->feedEvents(events);
    }
}

TEST_F(NetworkHandleTest, checkWeights) {

}

TEST_F(NetworkHandleTest, savingNetwork) {
    network->save(eventsPath, 1);
}

TEST_F(NetworkHandleTest, runningNetworkRL) {
    network = new NetworkHandle(networkPath2, eventsPath);

    while (network->loadEvents(events, 1)) {
        network->feedEvents(events);
    }
}

TEST_F(NetworkHandleTest, savingNetworkRL) {
    network->save(eventsPath, 1);
}

TEST_F(NetworkHandleTest, runningNetworkNoWeightSharing) {
    network = new NetworkHandle(networkPath3, eventsPath);

    while (network->loadEvents(events, 1)) {
        network->feedEvents(events);
    }
}

TEST_F(NetworkHandleTest, savingNetworkNoWeightSharing) {
    network->save(eventsPath, 1);
}