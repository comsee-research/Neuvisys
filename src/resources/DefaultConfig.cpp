//
// Created by thomas on 07/07/22.
//

#include "DefaultConfig.hpp"

namespace PredefinedConfigurations {
    NetConf oneLayerOnePatchNoWeightSharingConfig() {
        NetConf conf = {
                {
                        {"nbCameras",  1},
                        {"neuron1Synapses", 1},
                        {"sharingType", "none"},
                        {"neuronType",  {"SimpleCell"}},
                        {"inhibitions",  {{"static"}}},
                        {"interConnections", {{-1}}},
                        {"patches",  {{{93}, {50}, {0}}}},
                        {"size",    {{16, 16, 64}}},
                        {"neuronSizes", {{{10, 10, 1}}}},
                        {"neuronOverlap",     {{0, 0, 0}}},
                        {"neuronInhibitionRange", {1, 1}},
                        {"vfWidth",       346},
                        {"vfHeight",    260},
                        {"measurementInterval", 100}
                },
                {
                        {"rlTraining", false},
                        {"nu",              0.5},
                        {"V0",          0},
                        {"tauR",            1},
                        {"explorationFactor", 70},
                        {"actionRate",            500},
                        {"actionMapping", {{1,    5},         {1,   -5}}},
                        {"minActionRate", 100},
                        {"decayRate",   0.01},
                        {"intrinsicReward",   false},
                        {"scoreInterval",         2000}
                },
                {
                        {"VTHRESH",    30},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"POTENTIAL_TRACK", {4,            10}},
                        {"TAU_SRA",           100},
                        {"TAU_RP",                30},
                        {"TAU_M",         18},
                        {"TAU_LTP",       7},
                        {"TAU_LTD",     14},
                        {"TARGET_SPIKE_RATE", 0.75},
                        {"SYNAPSE_DELAY",         0},
                        {"STDP_LEARNING", "excitatory"},
                        {"NORM_FACTOR", 4},
                        {"LATERAL_NORM_FACTOR", 100},
                        {"TOPDOWN_NORM_FACTOR", 30},
                        {"DECAY_RATE", 0},
                        {"MIN_THRESH", 4},
                        {"ETA_LTP", 0.00077},
                        {"ETA_LTD", -0.00021},
                        {"ETA_ILTP", 7.7},
                        {"ETA_ILTD", -2.1},
                        {"ETA_SRA", 0.6},
                        {"ETA_TA", 0},
                        {"ETA_RP", 1},
                        {"ETA_INH", 20},
                },
                {
                        {"VTHRESH",    3},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"TAU_M",           20},
                        {"TAU_LTP",           20},
                        {"TAU_LTD",               20},
                        {"TAU_RP",        30},
                        {"STDP_LEARNING", "excitatory"},
                        {"NORM_FACTOR", 10},
                        {"DECAY_RATE",        0},
                        {"ETA_LTP",               0.2},
                        {"ETA_LTD",       0.2},
                        {"ETA_INH",     15},
                        {"ETA_RP",              1},
                },
                {
                        {"VTHRESH",    2},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"TAU_M",           20},
                        {"ETA_INH",           0},
                        {"TAU_LTP",               7},
                        {"TAU_LTD",       14},
                        {"ETA_LTP",       0.077},
                        {"ETA_LTD",     -0.021},
                        {"NORM_FACTOR",       10},
                        {"DECAY_RATE",            0},
                        {"STDP_LEARNING", "all"},
                        {"NU_K",        200},
                        {"MIN_NU_K",            100},
                        {"TAU_K",               50},
                        {"MIN_TAU_K",  25},
                        {"TAU_E",      500},
                        {"ETA",     0.2}
                },
                {
                        {"VTHRESH",    2},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"TAU_M",           20},
                        {"ETA_INH",           0},
                        {"TAU_LTP",               7},
                        {"TAU_LTD",       14},
                        {"ETA_LTP",       0.077},
                        {"ETA_LTD",     -0.021},
                        {"NORM_FACTOR",       10},
                        {"DECAY_RATE",            0},
                        {"STDP_LEARNING", "all"},
                        {"TAU_E",       250},
                        {"ETA",                 0.2}
                }
        };
        return conf;
    }

    NetConf twoLayerOnePatchWeightSharingCenteredConfig() {
        NetConf conf = {
                {
                        {"nbCameras",  1},
                        {"neuron1Synapses", 1},
                        {"sharingType", "full"},
                        {"neuronType",  {"SimpleCell", "ComplexCell"}},
                        {"inhibitions",  {{"static"}, {"static"}}},
                        {"interConnections", {{-1}, {0}}},
                        {"patches",  {{{93}, {50}, {0}}, {{0}, {0}, {0}}}},
                        {"size",    {{16, 16, 64}, {4, 4, 16}}},
                        {"neuronSizes", {{{10, 10, 1}}, {{4, 4, 64}}}},
                        {"neuronOverlap",     {{0, 0, 0}, {0, 0, 0}}},
                        {"neuronInhibitionRange", {1, 1}},
                        {"vfWidth",       346},
                        {"vfHeight",    260},
                        {"measurementInterval", 100}
                },
                {
                        {"rlTraining", false},
                        {"nu",              0.5},
                        {"V0",          0},
                        {"tauR",            1},
                        {"explorationFactor", 70},
                        {"actionRate",            500},
                        {"actionMapping", {{1,    5},         {1,   -5}}},
                        {"minActionRate", 100},
                        {"decayRate",   0.01},
                        {"intrinsicReward",   false},
                        {"scoreInterval",         2000}
                },
                {
                        {"VTHRESH",    30},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"POTENTIAL_TRACK", {4,            10}},
                        {"TAU_SRA",           100},
                        {"TAU_RP",                30},
                        {"TAU_M",         18},
                        {"TAU_LTP",       7},
                        {"TAU_LTD",     14},
                        {"TARGET_SPIKE_RATE", 0.75},
                        {"SYNAPSE_DELAY",         0},
                        {"STDP_LEARNING", "excitatory"},
                        {"NORM_FACTOR", 4},
                        {"LATERAL_NORM_FACTOR", 100},
                        {"TOPDOWN_NORM_FACTOR", 30},
                        {"DECAY_RATE", 0},
                        {"MIN_THRESH", 4},
                        {"ETA_LTP", 0.00077},
                        {"ETA_LTD", -0.00021},
                        {"ETA_ILTP", 7.7},
                        {"ETA_ILTD", -2.1},
                        {"ETA_SRA", 0.6},
                        {"ETA_TA", 0},
                        {"ETA_RP", 1},
                        {"ETA_INH", 20},
                },
                {
                        {"VTHRESH",    3},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"TAU_M",           20},
                        {"TAU_LTP",           20},
                        {"TAU_LTD",               20},
                        {"TAU_RP",        30},
                        {"STDP_LEARNING", "excitatory"},
                        {"NORM_FACTOR", 10},
                        {"DECAY_RATE",        0},
                        {"ETA_LTP",               0.2},
                        {"ETA_LTD",       0.2},
                        {"ETA_INH",     15},
                        {"ETA_RP",              1},
                },
                {
                        {"VTHRESH",    2},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"TAU_M",           20},
                        {"ETA_INH",           0},
                        {"TAU_LTP",               7},
                        {"TAU_LTD",       14},
                        {"ETA_LTP",       0.077},
                        {"ETA_LTD",     -0.021},
                        {"NORM_FACTOR",       10},
                        {"DECAY_RATE",            0},
                        {"STDP_LEARNING", "all"},
                        {"NU_K",        200},
                        {"MIN_NU_K",            100},
                        {"TAU_K",               50},
                        {"MIN_TAU_K",  25},
                        {"TAU_E",      500},
                        {"ETA",     0.2}
                },
                {
                        {"VTHRESH",    2},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"TAU_M",           20},
                        {"ETA_INH",           0},
                        {"TAU_LTP",               7},
                        {"TAU_LTD",       14},
                        {"ETA_LTP",       0.077},
                        {"ETA_LTD",     -0.021},
                        {"NORM_FACTOR",       10},
                        {"DECAY_RATE",            0},
                        {"STDP_LEARNING", "all"},
                        {"TAU_E",       250},
                        {"ETA",                 0.2}
                }
        };
        return conf;
    }

    NetConf fourLayerRLOnePatchCenteredConfig() {
        NetConf conf = {
                {
                        {"nbCameras",  1},
                        {"neuron1Synapses", 1},
                        {"sharingType", "full"},
                        {"neuronType",  {"SimpleCell", "ComplexCell", "CriticCell", "ActorCell"}},
                        {"inhibitions",  {{"static"}, {"static"}, {""}, {""}}},
                        {"interConnections", {{-1}, {0}, {0, 1}, {0, 1}}},
                        {"patches",  {{{93}, {50}, {0}}, {{0}, {0}, {0}}, {{0}, {0}, {0}}, {{0}, {0}, {0}}}},
                        {"size",    {{16, 16, 64}, {4, 4, 16}, {100, 1, 1}, {100, 1, 1}}},
                        {"neuronSizes", {{{10, 10, 1}}, {{4, 4, 64}}, {{16, 16, 64}, {4, 4, 16}}, {{16, 16, 64}, {4, 4, 16}}}},
                        {"neuronOverlap",     {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}},
                        {"neuronInhibitionRange", {1, 1}},
                        {"vfWidth",       346},
                        {"vfHeight",    260},
                        {"measurementInterval", 10}
                },
                {
                        {"rlTraining", true},
                        {"nu",              0.05},
                        {"V0",          0},
                        {"tauR",            1},
                        {"explorationFactor", 75},
                        {"actionRate",            200},
                        {"actionMapping", {{1,    5},         {1,   -5}}},
                        {"minActionRate", 10},
                        {"decayRate",   5},
                        {"intrinsicReward",   false},
                        {"scoreInterval",         2000}
                },
                {
                        {"VTHRESH",    30},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"POTENTIAL_TRACK", {4,            10}},
                        {"TAU_SRA",           100},
                        {"TAU_RP",                30},
                        {"TAU_M",         18},
                        {"TAU_LTP",       7},
                        {"TAU_LTD",     14},
                        {"TARGET_SPIKE_RATE", 0.75},
                        {"SYNAPSE_DELAY",         0},
                        {"STDP_LEARNING", "excitatory"},
                        {"NORM_FACTOR", 4},
                        {"LATERAL_NORM_FACTOR", 100},
                        {"TOPDOWN_NORM_FACTOR", 30},
                        {"DECAY_RATE", 0},
                        {"MIN_THRESH", 4},
                        {"ETA_LTP", 0.00077},
                        {"ETA_LTD", -0.00021},
                        {"ETA_ILTP", 7.7},
                        {"ETA_ILTD", -2.1},
                        {"ETA_SRA", 0.6},
                        {"ETA_TA", 0},
                        {"ETA_RP", 1},
                        {"ETA_INH", 20},
                },
                {
                        {"VTHRESH",    3},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"TAU_M",           20},
                        {"TAU_LTP",           20},
                        {"TAU_LTD",               20},
                        {"TAU_RP",        30},
                        {"STDP_LEARNING", "excitatory"},
                        {"NORM_FACTOR", 10},
                        {"DECAY_RATE",        0},
                        {"ETA_LTP",               0.2},
                        {"ETA_LTD",       0.2},
                        {"ETA_INH",     15},
                        {"ETA_RP",              1},
                },
                {
                        {"VTHRESH",    2},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"TAU_M",           20},
                        {"ETA_INH",           0},
                        {"TAU_LTP",               7},
                        {"TAU_LTD",       14},
                        {"ETA_LTP",       0.077},
                        {"ETA_LTD",     -0.021},
                        {"NORM_FACTOR",       10},
                        {"DECAY_RATE",            0},
                        {"STDP_LEARNING", "excitatory"},
                        {"NU_K",        200},
                        {"MIN_NU_K",            100},
                        {"TAU_K",               50},
                        {"MIN_TAU_K",  25},
                        {"TAU_E",      500},
                        {"ETA",     0.2}
                },
                {
                        {"VTHRESH",    2},
                        {"VRESET",          -20},
                        {"TRACKING",    "partial"},
                        {"TAU_M",           20},
                        {"ETA_INH",           0},
                        {"TAU_LTP",               7},
                        {"TAU_LTD",       14},
                        {"ETA_LTP",       0.077},
                        {"ETA_LTD",     -0.021},
                        {"NORM_FACTOR",       10},
                        {"DECAY_RATE",            0},
                        {"STDP_LEARNING", "excitatory"},
                        {"TAU_E",       250},
                        {"ETA",                 0.2}
                }
        };
        return conf;
    }
}