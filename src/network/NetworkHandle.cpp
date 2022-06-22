//
// Created by Thomas on 06/05/2021.
//

#include "NetworkHandle.hpp"

/**
 * Constructs an empty network.
 */
NetworkHandle::NetworkHandle() : m_saveTime(0) {

}

/**
 * Constucts the NetworkHandle without creating a SpikingNetwork object.
 * Used for reading event files.
 * @param eventsPath - Path to the event file.
 * @param time - Starting time for the network.
 */
NetworkHandle::NetworkHandle(const std::string &eventsPath, double time) : m_saveTime(time) {
    setEventPath(eventsPath);
}

/**
 * Constucts the NetworkHandle and creates a SpikingNetwork object associated to it.
 * Loads the configuration files locally.
 * Loads network weights if they exist.
 * @param networkPath - Path to the network folder.
 */
NetworkHandle::NetworkHandle(const std::string &networkPath) : m_spinet(networkPath),
                                                               m_rlConf(networkPath + "configs/rl_config.json"),
                                                               m_networkConf(NetworkConfig(networkPath + "configs/network_config.json")),
                                                               m_simpleNeuronConf(m_networkConf.getNetworkPath() + "configs/simple_cell_config.json",
                                                                                  0),
                                                               m_complexNeuronConf(
                                                                       m_networkConf.getNetworkPath() + "configs/complex_cell_config.json", 1),
                                                               m_criticNeuronConf(m_networkConf.getNetworkPath() + "configs/critic_cell_config.json",
                                                                                  2),
                                                               m_actorNeuronConf(m_networkConf.getNetworkPath() + "configs/actor_cell_config.json",
                                                                                 3),
                                                               m_saveTime(0) {
    load();
}

/**
 * Overload NetworkHandle constructor.
 * Loads an event file in addition to the usual constructor.
 * @param networkPath - Path to the network folder.
 * @param eventsPath - Path to the event file.
 */
NetworkHandle::NetworkHandle(const std::string &networkPath,
                             const std::string &eventsPath) : NetworkHandle(networkPath) {
    setEventPath(eventsPath);
}

void NetworkHandle::setEventPath(const std::string &eventsPath) {
    m_eventsPath = eventsPath;

    std::string hdf5 = ".h5";
    if (std::equal(hdf5.rbegin(), hdf5.rend(), m_eventsPath.rbegin())) {
        openH5File();
    }
}

/**
 * Open an HDF5 event file.
 */
void NetworkHandle::openH5File() {
    if (!m_eventsPath.empty()) {
        m_eventFile.file = H5::H5File(m_eventsPath, H5F_ACC_RDONLY);
        m_eventFile.group = m_eventFile.file.openGroup("events");
        m_eventFile.timestamps = m_eventFile.group.openDataSet("./t");
        m_eventFile.x = m_eventFile.group.openDataSet("./x");
        m_eventFile.y = m_eventFile.group.openDataSet("./y");
        m_eventFile.polarities = m_eventFile.group.openDataSet("./p");
        m_eventFile.cameras = m_eventFile.group.openDataSet("./c");
        m_eventFile.timestamps.getSpace().getSimpleExtentDims(&m_eventFile.dims);
        m_nbEvents += m_eventFile.dims;
        readFirstAndLastTimestamp();
    }
}

/**
 * Load packets of events from an event file.
 * Two format are recognized.
 * HDF5: loads events packet by packet.
 * NPZ: load all the events at the same time.
 * @param events - vector that will be filled with the events from the file.
 * @param nbPass - number of repetition of the event file.
 * @return true if all the events have been loaded. false otherwise.
 */
bool NetworkHandle::loadEvents(std::vector<Event> &events, size_t nbPass) {
    m_endTime = static_cast<double>(nbPass) * (getLastTimestamp() - getFirstTimestamp());
    std::string hdf5 = ".h5";
    std::string npz = ".npz";
    if (std::equal(hdf5.rbegin(), hdf5.rend(), m_eventsPath.rbegin())) {
        if (loadHDF5Events(events, nbPass)) {
            return false;
        }
        return true;
    } else if (std::equal(npz.rbegin(), npz.rend(), m_eventsPath.rbegin())) {
        if (m_iteration > 0) {
            return false;
        }
        loadNpzEvents(events, nbPass);
        return true;
    }
}

/**
 * Feed a packet of events to the network.
 * @param events - Vector of events.
 */
void NetworkHandle::feedEvents(const std::vector<Event> &events) {
    m_nbEvents = events.size();
    for (const auto &event: events) {
        ++m_iteration;
        transmitEvent(event);
        updateNeurons(event.timestamp());

        if (static_cast<double>(event.timestamp()) - m_saveTime.display > static_cast<size_t>(5 * E6)) {
            m_saveTime.display = static_cast<double>(event.timestamp());
            std::cout << static_cast<int>(static_cast<double>(100 * event.timestamp()) / m_endTime) << "%" << std::endl;
//            m_spinet.intermediateSave(m_saveCount);
//            ++m_saveCount;
        }
    }
}

/**
 *
 * @param time
 */
void NetworkHandle::updateNeurons(size_t time) {
    if (static_cast<double>(time) - m_saveTime.update > m_networkConf.getMeasurementInterval()) {
        m_totalNbEvents += m_countEvents;
        m_spinet.updateNeuronsStates(static_cast<long>(static_cast<double>(time) - m_saveTime.update));
        auto alpha = 0.6;
        m_averageEventRate = (alpha * static_cast<double>(m_countEvents)) + (1.0 - alpha) * m_averageEventRate;
        m_countEvents = 0;
        if (getRLConfig().getIntrinsicReward()) {
            m_reward = 100 * (2 - m_spinet.getAverageActivity()) / m_averageEventRate;
            m_spinet.transmitReward(m_reward);
        }
        m_saveData["eventRate"].push_back(m_averageEventRate);
        m_saveData["networkRate"].push_back(m_spinet.getAverageActivity());
        m_saveTime.update = static_cast<double>(time);
    }
}

/**
 *
 */
void NetworkHandle::resetAllNeurons() {
    for (int layer = 0; layer < m_networkConf.getLayerCellTypes().size(); layer++) {
        int numberOfNeuronsInLayer = m_networkConf.getLayerPatches()[layer][0].size() * m_networkConf.getLayerSizes()[layer][0] *
                                     m_networkConf.getLayerPatches()[layer][1].size() * m_networkConf.getLayerSizes()[layer][1] *
                                     m_networkConf.getLayerPatches()[layer][2].size() * m_networkConf.getLayerSizes()[layer][2];
        for (int j = 0; j < numberOfNeuronsInLayer; j++) {
            m_spinet.getNeuron(j, layer).get().resetNeuron();
        }
    }
}

/**
 *
 */
void NetworkHandle::load() {
    std::string fileName;
    fileName = getNetworkConfig().getNetworkPath() + "networkState.json";

    json state;
    std::ifstream ifs(fileName);
    if (ifs.is_open()) {
        try {
            ifs >> state;
            m_saveCount = state["save_count"];
            m_totalNbEvents = state["nb_events"];
        } catch (const std::exception &e) {
            std::cerr << "In network state file: " << fileName << e.what() << std::endl;
        }
    }
    m_spinet.loadWeights();
}

/**
 * Saves information about the network actual state.
 * @param eventFileName - Name of the event file.
 * @param nbRun - Number of time the event file has been shown.
 */
void NetworkHandle::save(const std::string &eventFileName = "", const size_t nbRun = 0) {
    std::cout << "Saving Network..." << std::endl;
    std::string fileName;
    fileName = m_networkConf.getNetworkPath() + "networkState.json";
    m_iteration = 0;
    json state;
    std::ifstream ifs(fileName);
//    resetAllNeurons();
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
    state["action_rate"] = getRLConfig().getActionRate();
    state["exploration_factor"] = getRLConfig().getExplorationFactor();
    state["save_count"] = m_saveCount;
    state["nb_events"] = m_totalNbEvents;

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

/**
 *
 * @param sequence
 */
void NetworkHandle::saveStatistics(size_t sequence) {
    std::cout << "Starting saving the statistics..." << std::endl;
    m_spinet.saveStatistics(sequence);
    resetAllNeurons();
    std::cout << "Finished." << std::endl;
}

/**
 *
 * @param lastTimestamp
 * @param time
 * @param nbEvents
 * @param msg
 * @return
 */
int NetworkHandle::learningLoop(long lastTimestamp, double time, size_t nbEvents, std::string &msg) {
    ++m_packetCount;
    saveValueMetrics(lastTimestamp, nbEvents);

    ++m_scoreCount;
    if (time - m_saveTime.console > m_rlConf.getScoreInterval()) {
        m_saveTime.console = time;
        learningDecay(m_rlConf.getScoreInterval() / E6);

        msg = "\n\nAverage reward: " + std::to_string(getScore(static_cast<long>(m_scoreCount))) +
              "\nExploration factor: " + std::to_string(getRLConfig().getExplorationFactor()) +
              "\nAction rate: " + std::to_string(getRLConfig().getActionRate());
        m_scoreCount = 0;
    }

    ++m_actionCount;
    if (time - m_saveTime.action > static_cast<double>(getRLConfig().getActionRate())) {
        m_saveTime.action = time;

        computeNeuromodulator();
        if (m_action != -1) {
            updateActor();
        }
        auto choice = actionSelection(resolveMotor(), getRLConfig().getExplorationFactor());
        m_action = choice.first;
        m_actionCount = 0;
        if (m_action != -1) {
            saveActionMetrics(m_action, choice.second);
            return m_action;
        }
    }
    return -1;
}

/**
 *
 * @param actionsActivations
 * @param explorationFactor
 * @return
 */
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

/**
 *
 */
void NetworkHandle::computeNeuromodulator() {
    double meanTDError = 0;
    auto count = 0;
    if (m_saveData["tdError"].size() > 10) {
        for (auto itTDError = m_saveData["tdError"].rbegin(); itTDError != m_saveData["tdError"].rend(); ++itTDError) {
            if (count > 10) {
                break;
            } else {
                meanTDError += *itTDError;
                ++count;
            }
        }
        m_neuromodulator = meanTDError / static_cast<double>(10);
    }
//    size_t layer = m_spinet.getNetworkStructure().size() - 2;
//    for (auto i = 0; i < m_spinet.getNetworkStructure()[layer]; ++i) { // critic cells
//        m_spinet.getNeuron(i, layer).get().setNeuromodulator(m_neuromodulator);
//    }
}

/**
 *
 */
void NetworkHandle::updateActor() {
    auto neuronPerAction = m_spinet.getNetworkStructure().back() / getRLConfig().getActionMapping().size();
    auto start = m_action * neuronPerAction;
    for (auto i = start; i < start + neuronPerAction; ++i) { // actor cells
        m_spinet.getNeuron(i, m_spinet.getNetworkStructure().size() - 1).get().setNeuromodulator(m_neuromodulator);
    }
//    std::cout << "Action: " << m_action + 1 << " TD: " << m_neuromodulator << std::endl;
//        m_spinet.normalizeActions();
}

/**
 *
 * @return
 */
std::vector<uint64_t> NetworkHandle::resolveMotor() {
    std::vector<uint64_t> motorActivations(getRLConfig().getActionMapping().size(), 0);

    auto neuronPerAction = m_spinet.getNetworkStructure().back() / getRLConfig().getActionMapping().size();
    size_t layer = m_spinet.getNetworkStructure().size() - 1;
    size_t action = 0, spikeCount = 0;
    for (size_t i = 0; i < m_spinet.getNetworkStructure().back(); ++i) {
        spikeCount += m_spinet.getNeuron(i, layer).get().getActivityCount();
        m_spinet.getNeuron(i, layer).get().resetActivityCount();
        if ((i + 1) % neuronPerAction == 0) {
            motorActivations[action] = spikeCount;
            spikeCount = 0;
            ++action;
        }
    }
    return motorActivations;
}

/**
 *
 * @param time
 * @param nbEvents
 */
void NetworkHandle::saveValueMetrics(long time, size_t nbEvents) {
    m_saveData["nbEvents"].push_back(static_cast<double>(nbEvents));
    m_saveData["reward"].push_back(m_reward);

    auto V = valueFunction(time);
    m_saveData["value"].push_back(V);
    double VDot = valueDerivative(m_saveData["value"]);
    m_saveData["valueDot"].push_back(VDot);
    auto tdError = VDot - V / getRLConfig().getTauR() + m_reward;
    m_saveData["tdError"].push_back(tdError);

    auto neuronPerAction = m_spinet.getNetworkStructure().back() / getRLConfig().getActionMapping().size();
    size_t action = 0, spikeCount = 0;
    for (size_t i = 0; i < m_spinet.getNetworkStructure().back(); ++i) {
        spikeCount += m_spinet.getNeuron(i, m_spinet.getNetworkStructure().size() - 1).get().getActivityCount();
        if ((i + 1) % neuronPerAction == 0) {
            m_saveData["action_" + std::to_string(action)].push_back(static_cast<double>(spikeCount));
            spikeCount = 0;
            ++action;
        }
    }
}

/**
 *
 * @param nbPreviousReward
 * @return
 */
double NetworkHandle::getScore(long nbPreviousReward) {
    double mean = 0;
    size_t count = 0;
    if (m_saveData["reward"].size() > nbPreviousReward) {
        for (auto it = m_saveData["reward"].rbegin(); it != m_saveData["reward"].rend(); ++it) {
            if (count > nbPreviousReward) {
                break;
            } else {
                mean += *it;
                ++count;
            }
        }
        mean /= static_cast<double>(nbPreviousReward);
        m_saveData["score"].push_back(mean);
    }
    return mean;
}

/**
 *
 * @param action
 * @param exploration
 */
void NetworkHandle::saveActionMetrics(size_t action, bool exploration) {
    m_saveData["action"].push_back(static_cast<double>(action));
    m_saveData["exploration"].push_back(exploration);
}

/**
 *
 * @param time
 * @return
 */
double NetworkHandle::valueFunction(long time) {
    double value = 0;
    size_t layer = m_spinet.getNetworkStructure().size() - 2;
    for (size_t i = 0; i < m_spinet.getNetworkStructure()[layer]; ++i) { // critic cells
        value += m_spinet.getNeuron(i, layer).get().updateKernelSpikingRate(time);
    }
    return getRLConfig().getNu() * value / static_cast<double>(m_spinet.getNetworkStructure()[layer]) + getRLConfig().getV0();
}

/**
 *
 * @param value
 * @return
 */
double NetworkHandle::valueDerivative(const std::vector<double> &value) {
    int nbPreviousTD = 100; // TODO : set a parameter
    if (value.size() > nbPreviousTD + 1) {
        return 100 * Util::secondOrderNumericalDifferentiationMean(value, nbPreviousTD);
    } else {
        return 0;
    }
}

/**
 *
 * @param reward
 */
void NetworkHandle::transmitReward(const double reward) {
    m_reward = reward;
    m_spinet.transmitReward(reward);
}

/**
 *
 * @param event
 */
void NetworkHandle::transmitEvent(const Event &event) {
    ++m_countEvents;
    m_spinet.addEvent(event);
}

/**
 *
 * @param time
 */
void NetworkHandle::learningDecay(double time) {
    double decay = time * getRLConfig().getDecayRate() / 100;

//    m_spinet.getNeuron(0, 2).get().learningDecay(1 - decay); // changing m_conf instance reference
//    m_spinet.getNeuron(0, 3).get().learningDecay(1 - decay);

    m_rlConf.setExplorationFactor(getRLConfig().getExplorationFactor() * (1 - decay));
//    if (getRLConfig().getActionRate() > getRLConfig().getMinActionRate()) {
//        getRLConfig().setActionRate(static_cast<long>(getRLConfig().getActionRate() * (1 - decay)));
//    }
}

/**
 *
 * @param time
 * @param id
 * @param layer
 */
void NetworkHandle::trackNeuron(const long time, const size_t id, const size_t layer) {
    if (m_simpleNeuronConf.TRACKING == "partial") {
        if (m_spinet.getNetworkStructure()[layer] > 0) {
            m_spinet.getNeuron(id, layer).get().trackPotential(time);
        }
    }
}

/**
 *
 * @param idNeuron
 * @param layer
 * @param camera
 * @param synapse
 * @param z
 * @return
 */
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

/**
 *
 * @param idNeuron
 * @param layer
 * @return
 */
cv::Mat NetworkHandle::getSummedWeightNeuron(size_t idNeuron, size_t layer) {
    if (m_spinet.getNetworkStructure()[layer] > 0) {
        return m_spinet.getNeuron(idNeuron, layer).get().summedWeightMatrix();
    }
    return cv::Mat::zeros(0, 0, CV_8UC3);
}

/**
 * Opens an event file (in the npz format) and load all the events in memory into a vector.
 * If nbPass is greater than 1, the events are concatenated multiple times and the timestamps are updated in accordance.
 * Returns the vector of events.
 * Only for loadNpzEvents camera event files.
 * @param events
 * @param nbPass
 */
void NetworkHandle::loadNpzEvents(std::vector<Event> &events, size_t nbPass) {
    events.clear();
    size_t pass, count;

    cnpy::NpyArray timestamps_array = cnpy::npz_load(m_eventsPath, "arr_0");
    cnpy::NpyArray x_array = cnpy::npz_load(m_eventsPath, "arr_1");
    cnpy::NpyArray y_array = cnpy::npz_load(m_eventsPath, "arr_2");
    cnpy::NpyArray polarities_array = cnpy::npz_load(m_eventsPath, "arr_3");
    size_t sizeArray = timestamps_array.shape[0];

    auto *ptr_timestamps = timestamps_array.data<long>();
    std::vector<long> timestamps(ptr_timestamps, ptr_timestamps + sizeArray);
    auto *ptr_x = x_array.data<int16_t>();
    std::vector<int16_t> x(ptr_x, ptr_x + sizeArray);
    auto *ptr_y = y_array.data<int16_t>();
    std::vector<int16_t> y(ptr_y, ptr_y + sizeArray);
    auto *ptr_polarities = polarities_array.data<bool>();
    std::vector<bool> polarities(ptr_polarities, ptr_polarities + sizeArray);

    auto cameras = std::vector<bool>(sizeArray, false);
    try {
        cnpy::NpyArray cameras_array = cnpy::npz_load(m_eventsPath, "arr_4");
        auto *ptr_cameras = cameras_array.data<bool>();
        cameras = std::vector<bool>(ptr_cameras, ptr_cameras + sizeArray);
    } catch (const std::exception &e) {
        std::cout << "NPZ file: " << e.what() << " defaulting to 1 camera.\n";
    }

    long firstTimestamp = timestamps[0];
    long lastTimestamp = static_cast<long>(timestamps[sizeArray - 1]);
    Event event{};

    for (pass = 0; pass < static_cast<size_t>(nbPass); ++pass) {
        for (count = 0; count < sizeArray; ++count) {
            event = Event(timestamps[count] + static_cast<long>(pass) * (lastTimestamp - firstTimestamp),
                          x[count],
                          y[count],
                          polarities[count],
                          cameras[count]);
            events.push_back(event);
        }
    }
    m_nbEvents = nbPass * sizeArray;
}

/**
 *
 * @param events
 * @return
 */
bool NetworkHandle::loadHDF5Events(std::vector<Event> &events, size_t nbPass) {
    if (m_eventFile.offset >= m_eventFile.dims) {
        m_eventFile.packetSize = 10000;
        ++m_eventFile.countPass;
        m_eventFile.offset = 0;
        if (m_eventFile.countPass >= nbPass) {
            return true;
        }
    } else if (m_eventFile.offset + m_eventFile.packetSize > m_eventFile.dims) {
        m_eventFile.packetSize = m_eventFile.dims - m_eventFile.offset;
    }

    events.clear();
    auto vT = std::vector<uint64_t>(m_eventFile.packetSize);
    auto vX = std::vector<uint16_t>(m_eventFile.packetSize);
    auto vY = std::vector<uint16_t>(m_eventFile.packetSize);
    auto vP = std::vector<uint8_t>(m_eventFile.packetSize);
    auto vC = std::vector<uint8_t>(m_eventFile.packetSize);

    H5::DataSpace filespace = m_eventFile.timestamps.getSpace();
    hsize_t dim[1] = {m_eventFile.packetSize};
    H5::DataSpace memspace(1, dim);
    filespace.selectHyperslab(H5S_SELECT_SET, &m_eventFile.packetSize, &m_eventFile.offset);

    m_eventFile.timestamps.read(vT.data(), H5::PredType::NATIVE_UINT64, memspace, filespace);
    m_eventFile.x.read(vX.data(), H5::PredType::NATIVE_UINT16, memspace, filespace);
    m_eventFile.y.read(vY.data(), H5::PredType::NATIVE_UINT16, memspace, filespace);
    m_eventFile.polarities.read(vP.data(), H5::PredType::NATIVE_UINT8, memspace, filespace);
    m_eventFile.cameras.read(vC.data(), H5::PredType::NATIVE_UINT8, memspace, filespace);

    auto vTOffset = m_eventFile.countPass * (m_eventFile.lastTimestamp - m_eventFile.firstTimestamp);
    for (int i = 0; i < m_eventFile.packetSize; i++) {
        if (getNetworkConfig().getNbCameras() == 2 || vC[i] == 0) {
            events.emplace_back(vT[i] - m_eventFile.firstTimestamp + vTOffset, vX[i], vY[i], vP[i], vC[i]);
        }
    }

    m_eventFile.offset += m_eventFile.packetSize;
    return false;
}

/**
 *
 */
void NetworkHandle::readFirstAndLastTimestamp() {
    H5::DataSpace filespace = m_eventFile.timestamps.getSpace();
    hsize_t dim[1] = {1};
    H5::DataSpace memspace(1, dim);
    hsize_t count = 1;

    hsize_t offset = 0;
    auto first = std::vector<uint64_t>(1);
    filespace.selectHyperslab(H5S_SELECT_SET, &count, &offset);
    m_eventFile.timestamps.read(first.data(), H5::PredType::NATIVE_UINT64, memspace, filespace);

    offset = m_eventFile.dims - 1;
    auto last = std::vector<uint64_t>(1);
    filespace.selectHyperslab(H5S_SELECT_SET, &count, &offset);
    m_eventFile.timestamps.read(last.data(), H5::PredType::NATIVE_UINT64, memspace, filespace);

    m_eventFile.firstTimestamp = first[0];
    m_eventFile.lastTimestamp = last[0];
}