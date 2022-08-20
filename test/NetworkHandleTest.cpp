//
// Created by thomas on 06/07/22.
//

#include "NetworkHandleTest.hpp"

NetworkHandle* NetworkHandleTest::network = nullptr;
std::string NetworkHandleTest::eventsPath;
std::string NetworkHandleTest::networkPath;
std::string NetworkHandleTest::networkPathComparison;
std::string NetworkHandleTest::networkPath2;
std::string NetworkHandleTest::networkPath3;

void NetworkHandleTest::SetUpTestSuite() {
    eventsPath = "../../data/events/shapes.h5";
    EXPECT_EQ(std::filesystem::exists("../../data/events/shapes.h5"), true);
    networkPath = "../../data/networks/network_test/";
    networkPathComparison = "../../data/networks/network_comparison_test/";
    networkPath2 = "../../data/networks/network_rl_test/";
    networkPath3 = "../../data/networks/network_nws_test/";

    NetworkConfig::createNetwork("../../data/networks/network_test", PredefinedConfigurations::twoLayerOnePatchWeightSharingCenteredConfig);
    NetworkConfig::createNetwork("../../data/networks/network_comparison_test", PredefinedConfigurations::twoLayerOnePatchWeightSharingCenteredConfig);
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
    std::filesystem::remove_all(networkPathComparison);
    std::filesystem::remove_all(networkPath2);
    std::filesystem::remove_all(networkPath2);
}

TEST_F(NetworkHandleTest, runningNetwork) {
    while (network->loadEvents(events, 1)) {
        network->feedEvents(events);
    }
}

TEST_F(NetworkHandleTest, checkWeightsInitialisationEquality) {
    network->save(eventsPath, 1);

    network = new NetworkHandle(networkPathComparison, eventsPath);
    network->save(eventsPath, 1);

    EXPECT_TRUE(compareWeights());
}

TEST_F(NetworkHandleTest, checkWeightsInitialisationDifference) {
    network->save(eventsPath, 1);

    WeightMatrix::setSeed(73114958);
    WeightMap::setSeed(92166846);
    network = new NetworkHandle(networkPathComparison, eventsPath);
    network->save(eventsPath, 1);

    EXPECT_FALSE(compareWeights());
}

TEST_F(NetworkHandleTest, checkWeightsEquality) {
    while (network->loadEvents(events, 1)) {
        network->feedEvents(events);
    }
    network->save(eventsPath, 1);

    network = new NetworkHandle(networkPathComparison, eventsPath);
    while (network->loadEvents(events, 1)) {
        network->feedEvents(events);
    }
    network->save(eventsPath, 1);

    EXPECT_TRUE(compareWeights());
}

TEST_F(NetworkHandleTest, checkWeightsDifference) {
    WeightMatrix::setSeed(68746166);
    WeightMap::setSeed(54684645);
    while (network->loadEvents(events, 1)) {
        network->feedEvents(events);
    }
    network->save(eventsPath, 1);

    WeightMatrix::setSeed(73114958);
    WeightMap::setSeed(92166846);
    network = new NetworkHandle(networkPathComparison, eventsPath);
    while (network->loadEvents(events, 1)) {
        network->feedEvents(events);
    }
    network->save(eventsPath, 1);

    EXPECT_FALSE(compareWeights());
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

bool NetworkHandleTest::compareWeights() {
    std::string path, pathComparison;
    for (size_t layer = 0; layer < network->getNetworkStructure().size(); ++layer) {
        for (size_t i = 0; i < network->getNetworkStructure()[layer]; ++i) {
            path = networkPath + "weights/" + std::to_string(layer) + "/" + std::to_string(i) + ".npy";
            pathComparison = networkPathComparison + "weights/" + std::to_string(layer) + "/" + std::to_string(i) + ".npy";
            if (Util::fileExist(path) && Util::fileExist(pathComparison)) {
                auto mat = WeightMatrix();
                mat.loadNumpyFile(path);

                auto matComparison = WeightMatrix();
                matComparison.loadNumpyFile(path);

                if (not(mat == matComparison)) {
                    return false;
                }
            }
        }
    }
    return true;
}