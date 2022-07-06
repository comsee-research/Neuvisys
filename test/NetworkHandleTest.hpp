//
// Created by thomas on 06/07/22.
//

#ifndef NEUVISYS_NETWORKHANDLETEST_HPP
#define NEUVISYS_NETWORKHANDLETEST_HPP

#include "gtest/gtest.h"
#include "../src/network/NetworkHandle.hpp"

class NetworkHandleTest : public ::testing::Test {
protected:
    static void SetUpTestSuite();

    static void TearDownTestSuite();

    void SetUp() override {};

    void TearDown() override {};

    static NetworkHandle *network;
    static std::string eventsPath;
    static std::string networkPath;
    Events events;
};


#endif //NEUVISYS_NETWORKHANDLETEST_HPP
