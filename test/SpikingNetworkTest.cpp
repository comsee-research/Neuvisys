//
// Created by thomas on 06/07/22.
//

#include "SpikingNetworkTest.hpp"

void SpikingNetworkTest::SetUp() {
    NetworkConfig::createNetwork("/home/thomas/network_test");
    spinet = SpikingNetwork("/home/thomas/network_test/");
}

void SpikingNetworkTest::TearDown() {
    std::filesystem::remove_all("/home/thomas/network_test/");
}

TEST_F(SpikingNetworkTest, folderCreation) {
    EXPECT_EQ(std::filesystem::exists("/home/thomas/network_test"), true);
}

TEST_F(SpikingNetworkTest, networkCreation) {
    EXPECT_EQ(spinet.getNetworkStructure()[0], 16*16*64);
    EXPECT_EQ(spinet.getNetworkStructure()[1], 4*4*16);
}