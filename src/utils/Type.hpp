//
// Created by thomas on 06/07/22.
//

#ifndef NEUVISYS_TYPE_HPP
#define NEUVISYS_TYPE_HPP

#include "../network/Event.hpp"
#include "../dependencies/json.hpp"
#include <vector>
#include <unordered_map>
#include <array>
#include <queue>
#include <stack>
#include <boost/circular_buffer.hpp>
#include "H5Cpp.h"
#include "hdf5.h"

using Events = std::vector<Event>;
using NetConf = std::vector<nlohmann::json>;

#endif //NEUVISYS_TYPE_HPP
