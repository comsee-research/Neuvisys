//
// Created by thomas on 07/07/22.
//

#ifndef NEUVISYS_DEFAULTCONFIG_HPP
#define NEUVISYS_DEFAULTCONFIG_HPP

#include "../utils/Type.hpp"

namespace PredefinedConfigurations {
    NetConf oneLayerOnePatchNoWeightSharingConfig();
    NetConf twoLayerOnePatchWeightSharingCenteredConfig();
    NetConf fourLayerRLOnePatchCenteredConfig();
}

#endif //NEUVISYS_DEFAULTCONFIG_HPP
