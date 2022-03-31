//
// Created by alphat on 14/04/2021.
//

#include "NetworkHandle.hpp"

#include <utility>

NetworkHandle::NetworkHandle() : m_saveTime(0) {

}

NetworkHandle::NetworkHandle(std::string leftEventsPath,
                             std::string rightEventsPath) : m_saveTime(0),
                                                            m_leftEventsPath(std::move(leftEventsPath)),
                                                            m_rightEventsPath(std::move(rightEventsPath)) {
    std::string hdf5 = ".h5";
    if (std::equal(hdf5.rbegin(), hdf5.rend(), m_leftEventsPath.rbegin())) {
        loadH5File();
    }
}

/* Creates the spiking neural network from the SpikingNetwork class.
 * Loads the configuration files locally.
 * Loads possible network weights if the network has already been created and saved before.
 */
NetworkHandle::NetworkHandle(const std::string &networkPath,
                             double time) : m_spinet(networkPath),
                                            m_networkConf(NetworkConfig(networkPath)),
                                            m_simpleNeuronConf(
                                                    m_networkConf.getNetworkPath() +
                                                    "configs/simple_cell_config.json",
                                                    0),
                                            m_complexNeuronConf(
                                                    m_networkConf.getNetworkPath() +
                                                    "configs/complex_cell_config.json",
                                                    1),
                                            m_criticNeuronConf(
                                                    m_networkConf.getNetworkPath() +
                                                    "configs/critic_cell_config.json",
                                                    2),
                                            m_actorNeuronConf(
                                                    m_networkConf.getNetworkPath() +
                                                    "configs/actor_cell_config.json",
                                                    3),
                                            m_saveTime(time) {
    load();
}

NetworkHandle::NetworkHandle(const std::string &networkPath, double time, const std::string &leftEventsPath,
                             const std::string &rightEventsPath) : NetworkHandle(networkPath, time) {
    m_leftEventsPath = leftEventsPath;
    m_rightEventsPath = rightEventsPath;

    std::string hdf5 = ".h5";
    if (std::equal(hdf5.rbegin(), hdf5.rend(), m_leftEventsPath.rbegin())) {
        loadH5File();
    }
}

void NetworkHandle::loadH5File() {
    char *version, *date;
    register_blosc(&version, &date);

    if (!m_leftEventsPath.empty()) {
        m_leftEvents.file = H5::H5File(m_leftEventsPath, H5F_ACC_RDONLY);
        m_leftEvents.group = m_leftEvents.file.openGroup("events");
        m_leftEvents.timestamps = m_leftEvents.group.openDataSet("./t");
        m_leftEvents.x = m_leftEvents.group.openDataSet("./x");
        m_leftEvents.y = m_leftEvents.group.openDataSet("./y");
        m_leftEvents.polarities = m_leftEvents.group.openDataSet("./p");
        m_leftEvents.timestamps.getSpace().getSimpleExtentDims(&m_leftEvents.dims);
        m_nbEvents += m_leftEvents.dims;
    }
    if (!m_rightEventsPath.empty()) {
        m_rightEvents.file = H5::H5File(m_leftEventsPath, H5F_ACC_RDONLY);
        m_rightEvents.group = m_rightEvents.file.openGroup("events");
        m_rightEvents.timestamps = m_rightEvents.group.openDataSet("./t");
        m_rightEvents.x = m_rightEvents.group.openDataSet("./x");
        m_rightEvents.y = m_rightEvents.group.openDataSet("./y");
        m_rightEvents.polarities = m_rightEvents.group.openDataSet("./p");
        m_rightEvents.timestamps.getSpace().getSimpleExtentDims(&m_rightEvents.dims);
        m_nbEvents += m_rightEvents.dims;
    }
}

bool NetworkHandle::loadEvents(std::vector<Event> &events, size_t nbPass) {
    std::string hdf5 = ".h5";
    if (std::equal(hdf5.rbegin(), hdf5.rend(), m_leftEventsPath.rbegin())) {
        if (loadHDF5Events(events)) {
            return false;
        }
        return true;
    } else {
        if (m_iteration > 0) {
            return false;
        }
        if (m_networkConf.getNbCameras() == 1) {
            mono(events, nbPass);
        } else if (m_networkConf.getNbCameras() == 2) {
            stereo(events, nbPass);
        }
        return true;
    }
}

/* Launches the spiking network on the specified event file 'nbPass' times.
 * The config file of the network must precise if the event file is stereo or mono.
 */
void NetworkHandle::feedEvents(const std::vector<Event> &events) {
    for (const auto &event: events) {
        ++m_iteration;
        transmitEvent(event);

        if (static_cast<double>(event.timestamp()) - m_saveTime.update > UPDATE_INTERVAL) {
            m_saveTime.update = static_cast<double>(event.timestamp());
            m_spinet.updateNeuronsStates(UPDATE_INTERVAL, m_countEvents);
        }

        if (static_cast<double>(event.timestamp()) - m_saveTime.display > static_cast<size_t>(5 * E6)) {
            m_saveTime.display = static_cast<double>(event.timestamp());
            std::cout << static_cast<int>(100 * m_iteration / m_nbEvents) << "%" << std::endl;
//            m_spinet.intermediateSave(m_saveCount);
//            ++m_saveCount;
        }
    }
}

void NetworkHandle::load() {
    std::string fileName;
    fileName = m_networkConf.getNetworkPath() + "networkState.json";

    json state;
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> state;
            m_saveCount = state["save_count"];
            m_countEvents = state["nb_events"];
        } catch (const std::exception &e) {
            std::cerr << "In network state file: " << fileName << e.what() << std::endl;
        }
    }

    m_spinet.loadWeights();
}

/* Saves information about the network actual state.
 * Number of times the network has been launched and the name of the event file use can be specified.
 */
void NetworkHandle::save(const std::string &eventFileName = "", const size_t nbRun = 0) {
    std::cout << "Saving Network..." << std::endl;
    std::string fileName;
    fileName = m_networkConf.getNetworkPath() + "networkState.json";

    json state;
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> state;
            auto eventFileNames = static_cast<std::vector<std::string>>(state["event_file_name"]);
            auto nbRuns = static_cast<std::vector<std::size_t>>(state["nb_run"]);
            eventFileNames.push_back(eventFileName);
            nbRuns.push_back(nbRun);
            state["event_file_name"] = eventFileNames;
            state["nb_run"] = nbRuns;
        } catch (const std::exception &e) {
            std::cerr << "In network state file: " << fileName << e.what() << std::endl;
        }
    } else {
        std::cout << "Creating network state file" << std::endl;
        std::vector<std::string> fileNames = {eventFileName};
        std::vector<size_t> nbRuns = {nbRun};
        state["event_file_name"] = fileNames;
        state["nb_run"] = nbRuns;
    }

    state["learning_data"] = m_saveData;
    state["action_rate"] = m_networkConf.getActionRate();
    state["exploration_factor"] = m_networkConf.getExplorationFactor();
    state["save_count"] = m_saveCount;
    state["nb_events"] = m_countEvents;

    std::ofstream ofs(fileName);
    if (ofs.is_open()) {
        ofs << std::setw(4) << state << std::endl;
    } else {
        std::cout << "cannot save network state file" << std::endl;
    }
    ofs.close();

    m_spinet.saveNetwork();

    std::cout << "Finished." << std::endl;
}

int NetworkHandle::learningLoop(long lastTimestamp, double time, size_t nbEvents, std::string &msg) {
    if (time - m_saveTime.update > static_cast<double>(UPDATE_INTERVAL)) {
        m_saveTime.update = time;
        m_spinet.updateNeuronsStates(UPDATE_INTERVAL, m_countEvents);
        m_reward = 50 * m_spinet.getAverageActivity();
        m_spinet.transmitNeuromodulator(m_reward);
    }

    saveValueMetrics(lastTimestamp, nbEvents);

    if (time - m_saveTime.console > SCORE_INTERVAL) {
        m_saveTime.console = time;
        learningDecay(m_iteration);
        ++m_iteration;
        msg = "\n\nAverage reward: " + std::to_string(getScore(SCORE_INTERVAL * E3 / DT)) +
              "\nExploration factor: " + std::to_string(getNetworkConfig().getExplorationFactor()) +
              "\nAction rate: " + std::to_string(getNetworkConfig().getActionRate());
    }

    if (time - m_saveTime.action > static_cast<double>(getNetworkConfig().getActionRate())) {
        m_saveTime.action = time;
        if (m_action != -1) {
            updateActor(lastTimestamp, m_action);
        }
        auto choice = actionSelection(resolveMotor(), getNetworkConfig().getExplorationFactor());
        m_action = choice.first;
        if (m_action != -1) {
            saveActionMetrics(m_action, choice.second);
            return m_action;
        }
    }
    return -1;
}

void NetworkHandle::updateActor(long time, size_t actor) {
    auto neuron = m_spinet.getNeuron(actor, m_spinet.getNetworkStructure().size() - 1);
    neuron.get().spike(time);

    neuron.get().setNeuromodulator(m_saveData["tdError"].back());
    neuron.get().weightUpdate(); // TODO: what about the eligibility traces (previous action) ?
    m_spinet.normalizeActions();
}

void NetworkHandle::saveValueMetrics(long time, size_t nbEvents) {
    m_saveData["nbEvents"].push_back(static_cast<double>(nbEvents));
    m_saveData["reward"].push_back(m_reward);

    auto V = valueFunction(time);
    m_saveData["value"].push_back(V);
    double VDot = valueDerivative(m_saveData["value"]);
    m_saveData["valueDot"].push_back(VDot);
    auto tdError = VDot - V / m_networkConf.getTauR() + m_reward;
    m_saveData["tdError"].push_back(tdError);
}

double NetworkHandle::getScore(long time) {
    double mean = 0;
    for (auto it = m_saveData["reward"].end();
         it != m_saveData["reward"].end() - time && it != m_saveData["reward"].begin(); --it) {
        mean += *it;
    }
    mean /= static_cast<double>(time);
    m_saveData["score"].push_back(mean);
    return mean;
}

void NetworkHandle::saveActionMetrics(size_t action, bool exploration) {
    m_saveData["action"].push_back(static_cast<double>(action));
    m_saveData["exploration"].push_back(exploration);
}

double NetworkHandle::valueFunction(long time) {
    double value = 0;
    for (size_t i = 0; i < m_spinet.getNetworkStructure()[2]; ++i) {
        value += m_spinet.getNeuron(i, 2).get().updateKernelSpikingRate(time);
    }
    return m_networkConf.getNu() * value / static_cast<double>(m_spinet.getNetworkStructure()[2]) +
           m_networkConf.getV0();
}

double NetworkHandle::valueDerivative(const std::vector<double> &value) {
    int nbPreviousTD = static_cast<int>(m_networkConf.getActionRate() / E3) / DT;
    if (value.size() > nbPreviousTD) {
        return 50 * Util::secondOrderNumericalDifferentiationMean(value, nbPreviousTD);
    } else {
        return 0;
    }
}

void NetworkHandle::transmitReward(const double reward) {
    m_reward = reward;
    m_spinet.transmitNeuromodulator(reward);
}

void NetworkHandle::transmitEvent(const Event &event) {
    ++m_countEvents;
    m_spinet.addEvent(event);
}

std::vector<uint64_t> NetworkHandle::resolveMotor() {
    std::vector<uint64_t> motorActivations(m_spinet.getNetworkStructure().back(), 0);

    size_t layer = m_spinet.getNetworkStructure().size() - 1;
    for (size_t i = 0; i < m_spinet.getNetworkStructure().back(); ++i) {
        motorActivations[m_spinet.getNeuron(i, layer).get().getIndex()] = m_spinet.getNeuron(i,
                                                                                             layer).get().getActivityCount();
    }
    return motorActivations;
}

void NetworkHandle::learningDecay(size_t iteration) {
    double decay = 1 + getNetworkConfig().getDecayRate() * static_cast<double>(iteration);

    m_spinet.getNeuron(0, 2).get().learningDecay(decay); // changing m_conf instance reference
    m_spinet.getNeuron(0, 3).get().learningDecay(decay);

    m_networkConf.setExplorationFactor(m_networkConf.getExplorationFactor() / decay);
    if (m_networkConf.getActionRate() > m_networkConf.getMinActionRate()) {
        m_networkConf.setActionRate(static_cast<long>(m_networkConf.getActionRate() / decay));
    }
}

void NetworkHandle::trackNeuron(const long time, const size_t id, const size_t layer) {
    if (m_simpleNeuronConf.TRACKING == "partial") {
        if (m_spinet.getNetworkStructure()[layer] > 0) {
            m_spinet.getNeuron(id, layer).get().trackPotential(time);
        }
    }
}

cv::Mat NetworkHandle::getWeightNeuron(size_t idNeuron, size_t layer, size_t camera, size_t synapse, size_t z) {
    if (m_spinet.getNetworkStructure()[layer] > 0) {
        auto dim = m_spinet.getNeuron(0, layer).get().getWeightsDimension();
        cv::Mat weightImage = cv::Mat::zeros(static_cast<int>(dim[1]), static_cast<int>(dim[0]), CV_8UC3);

        double weight;
        for (size_t x = 0; x < dim[0]; ++x) {
            for (size_t y = 0; y < dim[1]; ++y) {
                if (layer == 0) {
                    for (size_t p = 0; p < NBPOLARITY; p++) {
                        weight = m_spinet.getNeuron(idNeuron, layer).get().getWeights(p, camera, synapse, x, y) * 255;
                        weightImage.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x))[static_cast<int>(2 -
                                                                                                             p)] = static_cast<unsigned char>
                        (weight);
                    }
                } else {
                    weight = m_spinet.getNeuron(idNeuron, layer).get().getWeights(x, y, z) * 255;
                    weightImage.at<cv::Vec3b>(static_cast<int>(y),
                                              static_cast<int>(x))[0] = static_cast<unsigned char>(weight);
                }
            }
        }
        double min, max;
        minMaxIdx(weightImage, &min, &max);
        cv::Mat weights = weightImage * 255 / max;
        return weights;
    }
    return cv::Mat::zeros(0, 0, CV_8UC3);
}

cv::Mat NetworkHandle::getSummedWeightNeuron(size_t idNeuron, size_t layer) {
    if (m_spinet.getNetworkStructure()[layer] > 0) {
        return m_spinet.getNeuron(idNeuron, layer).get().summedWeightMatrix();
    }
    return cv::Mat::zeros(0, 0, CV_8UC3);
}

std::pair<int, bool>
NetworkHandle::actionSelection(const std::vector<uint64_t> &actionsActivations, const double explorationFactor) {
    bool exploration = false;
    int selectedAction = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distReal(0.0, 1.0);
    std::uniform_int_distribution<> distInt(0, static_cast<int>(actionsActivations.size() - 1));

    auto real = 100 * distReal(gen);
    if (real >= explorationFactor) {
        selectedAction = Util::winnerTakeAll(actionsActivations);
    } else {
        selectedAction = distInt(gen);
        exploration = true;
    }

    return std::make_pair(selectedAction, exploration);
}

/* Opens an event file (in the npz format) and load all the events in memory into a vector.
 * If nbPass is greater than 1, the events are concatenated multiple times and the timestamps are updated in accordance.
 * Returns the vector of events.
 * Only for mono camera event files.
 */
void NetworkHandle::mono(std::vector<Event> &events, size_t nbPass) {
    events.clear();
    size_t pass, count;

    cnpy::NpyArray timestamps_array = cnpy::npz_load(m_leftEventsPath, "arr_0");
    cnpy::NpyArray x_array = cnpy::npz_load(m_leftEventsPath, "arr_1");
    cnpy::NpyArray y_array = cnpy::npz_load(m_leftEventsPath, "arr_2");
    cnpy::NpyArray polarities_array = cnpy::npz_load(m_leftEventsPath, "arr_3");
    size_t sizeLeftArray = timestamps_array.shape[0];

    auto *l_timestamps = timestamps_array.data<long>();
    auto *l_x = x_array.data<int16_t>();
    auto *l_y = y_array.data<int16_t>();
    auto *l_polarities = polarities_array.data<bool>();

    long firstTimestamp = l_timestamps[0];
    long lastTimestamp = static_cast<long>(l_timestamps[sizeLeftArray - 1]);
    Event event{};

    for (pass = 0; pass < static_cast<size_t>(nbPass); ++pass) {
        for (count = 0; count < sizeLeftArray; ++count) {
            event = Event(l_timestamps[count] + static_cast<long>(pass) * (lastTimestamp - firstTimestamp), l_x[count],
                          l_y[count],
                          l_polarities[count], 0);
            events.push_back(event);
        }
    }
}

/* Same as mono function but for stereo camera event files.
 */
void NetworkHandle::stereo(std::vector<Event> &events, size_t nbPass) {
    events.clear();
    size_t pass, left, right;

    cnpy::NpyArray lTimestampsArray = cnpy::npz_load(m_leftEventsPath, "arr_0");
    cnpy::NpyArray lXArray = cnpy::npz_load(m_leftEventsPath, "arr_1");
    cnpy::NpyArray lYArray = cnpy::npz_load(m_leftEventsPath, "arr_2");
    cnpy::NpyArray lPolaritiesArray = cnpy::npz_load(m_leftEventsPath, "arr_3");
    size_t sizeLeftArray = lTimestampsArray.shape[0];

    auto *lTimestamps = lTimestampsArray.data<long>();
    auto *lX = lXArray.data<int16_t>();
    auto *lY = lYArray.data<int16_t>();
    auto *lPolarities = lPolaritiesArray.data<bool>();

    cnpy::NpyArray rTimestampsArray = cnpy::npz_load(m_leftEventsPath, "arr_4");
    cnpy::NpyArray rXArray = cnpy::npz_load(m_leftEventsPath, "arr_5");
    cnpy::NpyArray rYArray = cnpy::npz_load(m_leftEventsPath, "arr_6");
    cnpy::NpyArray rPolaritiesArray = cnpy::npz_load(m_leftEventsPath, "arr_7");
    size_t sizeRightArray = rTimestampsArray.shape[0];

    auto *rTimestamps = rTimestampsArray.data<long>();
    auto *rX = rXArray.data<int16_t>();
    auto *rY = rYArray.data<int16_t>();
    auto *rPolarities = rPolaritiesArray.data<bool>();

    long max = std::max(lTimestamps[sizeLeftArray - 1], rTimestamps[sizeRightArray - 1]);
    long leftInterval = max - lTimestamps[0];
    long rightInterval = max - rTimestamps[0];

    long lT, rT;
    Event event{};

    for (pass = 0; pass < static_cast<size_t>(nbPass); ++pass) {
        left = 0;
        right = 0;
        while (left < sizeLeftArray || right < sizeRightArray) {
            lT = lTimestamps[left] + static_cast<long>(pass) * leftInterval;
            rT = rTimestamps[right] + static_cast<long>(pass) * rightInterval;
            if ((left < sizeLeftArray) && (lT <= rT || right >= sizeRightArray)) {
                events.emplace_back(lT, lX[left], lY[left], lPolarities[left], 0);
                ++left;
            } else if ((right < sizeRightArray) && (lT > rT || left >= sizeLeftArray)) {
                events.emplace_back(rT, rX[right], rY[right], rPolarities[right], 1);
                ++right;
            }
        }
    }
}

bool NetworkHandle::loadHDF5Events(std::vector<Event> &events) {
    if (m_leftEvents.offset > m_leftEvents.dims) {
        return true;
    }

    events.clear();
    auto vT = std::vector<uint64_t>(m_leftEvents.packetSize);
    auto vX = std::vector<uint16_t>(m_leftEvents.packetSize);
    auto vY = std::vector<uint16_t>(m_leftEvents.packetSize);
    auto vP = std::vector<uint8_t>(m_leftEvents.packetSize);

    H5::DataSpace filespace = m_leftEvents.timestamps.getSpace();
    hsize_t dim[1] = {m_leftEvents.packetSize};
    H5::DataSpace memspace(1, dim);
    filespace.selectHyperslab(H5S_SELECT_SET, &m_leftEvents.packetSize, &m_leftEvents.offset);

    m_leftEvents.timestamps.read(vT.data(), H5::PredType::NATIVE_UINT64, memspace, filespace);
    m_leftEvents.x.read(vX.data(), H5::PredType::NATIVE_UINT16, memspace, filespace);
    m_leftEvents.y.read(vY.data(), H5::PredType::NATIVE_UINT16, memspace, filespace);
    m_leftEvents.polarities.read(vP.data(), H5::PredType::NATIVE_UINT8, memspace, filespace);

    for (int i = 0; i < m_leftEvents.packetSize; i++) {
        if (vX[i] < 346 && vY[i] < 260) {
            events.emplace_back(vT[i], vX[i], vY[i], vP[i], 0);
        }
    }

    m_leftEvents.offset += m_leftEvents.packetSize; // TODO: do not cut the last events
    return false;
}

bool NetworkHandle::loadHDF5EventsStereo(std::vector<Event> &events) {
    if (m_leftEvents.offset > m_leftEvents.dims || m_rightEvents.offset > m_rightEvents.dims) { // TODO: account for all events
        return true;
    }

    events.clear();
    auto lT = std::vector<uint64_t>(m_leftEvents.packetSize);
    auto lX = std::vector<uint16_t>(m_leftEvents.packetSize);
    auto lY = std::vector<uint16_t>(m_leftEvents.packetSize);
    auto lP = std::vector<uint8_t>(m_leftEvents.packetSize);

    auto rT = std::vector<uint64_t>(m_leftEvents.packetSize);
    auto rX = std::vector<uint16_t>(m_leftEvents.packetSize);
    auto rY = std::vector<uint16_t>(m_leftEvents.packetSize);
    auto rP = std::vector<uint8_t>(m_leftEvents.packetSize);

    H5::DataSpace lFilespace = m_leftEvents.timestamps.getSpace();
    hsize_t  lDim[1] = {m_leftEvents.packetSize};
    H5::DataSpace lMemspace(1, lDim);


    H5::DataSpace rFilespace = m_rightEvents.timestamps.getSpace();
    hsize_t  rDim[1] = {m_rightEvents.packetSize};
    H5::DataSpace rMemspace(1, lDim);

    lFilespace.selectHyperslab(H5S_SELECT_SET, &m_leftEvents.packetSize, &m_leftEvents.offset);
    rFilespace.selectHyperslab(H5S_SELECT_SET, &m_rightEvents.packetSize, &m_rightEvents.offset);

    m_leftEvents.timestamps.read(lT.data(), H5::PredType::NATIVE_UINT64, lMemspace, lFilespace);
    m_leftEvents.x.read(lX.data(), H5::PredType::NATIVE_UINT16, lMemspace, lFilespace);
    m_leftEvents.y.read(lY.data(), H5::PredType::NATIVE_UINT16, lMemspace, lFilespace);
    m_leftEvents.polarities.read(lP.data(), H5::PredType::NATIVE_UINT8, lMemspace, lFilespace);

    m_rightEvents.timestamps.read(rT.data(), H5::PredType::NATIVE_UINT64, rMemspace, rFilespace);
    m_rightEvents.x.read(rX.data(), H5::PredType::NATIVE_UINT16, rMemspace, rFilespace);
    m_rightEvents.y.read(rY.data(), H5::PredType::NATIVE_UINT16, rMemspace, rFilespace);
    m_rightEvents.polarities.read(rP.data(), H5::PredType::NATIVE_UINT8, rMemspace, rFilespace);

    size_t left = 0;
    size_t right = 0;
    Event event{};
    while (left < m_leftEvents.packetSize || right < m_rightEvents.packetSize) {
        if ((left < m_leftEvents.packetSize) && (lT[left] <= rT[right] || right >= m_rightEvents.packetSize)) {
            events.emplace_back(lT[left], lX[left], lY[left], lP[left], 0);
            ++left;
        } else if ((right < m_rightEvents.packetSize) && (lT[left] > rT[right] || left >= m_leftEvents.packetSize)) {
            events.emplace_back(rT[right], rX[right], rY[right], rP[right], 1);
            ++right;
        }
    }

    m_leftEvents.offset += m_leftEvents.packetSize;
    m_rightEvents.offset += m_rightEvents.packetSize;
    return false;
}