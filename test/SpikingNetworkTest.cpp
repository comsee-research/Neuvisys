//
// Created by thomas on 06/07/22.
//

#include "SpikingNetworkTest.hpp"

void SpikingNetworkTest::SetUp() {
    NetworkConfig::createNetwork("../../src/resources/network_test", PredefinedConfigurations::twoLayerOnePatchCenteredConfig);
    spinet = SpikingNetwork("../../src/resources/network_test/");
}

void SpikingNetworkTest::TearDown() {
    std::filesystem::remove_all("../../src/resources/network_test/");
}

TEST_F(SpikingNetworkTest, folderCreation) {
    EXPECT_EQ(std::filesystem::exists("../../src/resources/network_test"), true);
}

TEST_F(SpikingNetworkTest, networkCreation) {
    EXPECT_EQ(spinet.getNetworkStructure()[0], 16*16*64);
    EXPECT_EQ(spinet.getNetworkStructure()[1], 4*4*16);
}