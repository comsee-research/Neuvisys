//
// Created by alphat on 14/04/2021.
//

#include "NetworkHandle.hpp"
#include <utility>

NetworkHandle::NetworkHandle() : m_saveTime(0) {

}

NetworkHandle::NetworkHandle(std::string eventsPath, double time) : m_saveTime(time), m_eventsPath(std::move(eventsPath)) {
    std::string hdf5 = ".h5";
    if (std::equal(hdf5.rbegin(), hdf5.rend(), m_eventsPath.rbegin())) {
        loadH5File();
    }
}

/* Creates the spiking neural network from the SpikingNetwork class.
 * Loads the configuration files locally.
 * Loads possible network weights if the network has already been created and saved before.
 */
NetworkHandle::NetworkHandle(const std::string &networkPath) : m_spinet(networkPath),
                                            m_networkConf(NetworkConfig(networkPath + "configs/network_config.json")),
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
                                            m_saveTime(0) {
    load();
}

NetworkHandle::NetworkHandle(const std::string &networkPath,
                             const std::string &eventsPath) : NetworkHandle(networkPath) {
    m_eventsPath = eventsPath;

    std::string hdf5 = ".h5";
    if (std::equal(hdf5.rbegin(), hdf5.rend(), m_eventsPath.rbegin())) {
        loadH5File();
    }
}

void NetworkHandle::loadH5File() {
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
    }
}

bool NetworkHandle::loadEvents(std::vector<Event> &events, size_t nbPass) {
    std::string hdf5 = ".h5";
    std::string npz = ".npz";
    if (std::equal(hdf5.rbegin(), hdf5.rend(), m_eventsPath.rbegin())) {
        if (loadHDF5Events(events)) {
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

/* Launches the spiking network on the specified event file 'nbPass' times.
 * The config file of the network must precise if the event file is stereo or loadNpzEvents.
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
//        m_reward = 50 * m_spinet.getAverageActivity();
//        m_spinet.transmitNeuromodulator(m_reward);
    }

    saveValueMetrics(lastTimestamp, nbEvents);

    if (time - m_saveTime.console > SCORE_INTERVAL) {
        m_saveTime.console = time;
        learningDecay(m_scoreIteration);
        ++m_scoreIteration;
        msg = "\n\nAverage reward: " + std::to_string(getScore(SCORE_INTERVAL / m_timeStep)) +
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

void NetworkHandle::updateActor(long time, size_t actor) {
    auto neuron = m_spinet.getNeuron(actor, m_spinet.getNetworkStructure().size() - 1);
    neuron.get().spike(time);

    neuron.get().setNeuromodulator(m_saveData["tdError"].back());
    neuron.get().weightUpdate(); // TODO: what about the eligibility traces (previous action) ?
    m_spinet.normalizeActions();
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

double NetworkHandle::getScore(long nbPreviousReward) {
    double mean = 0;
    for (auto it = m_saveData["reward"].end();
         it != m_saveData["reward"].end() - nbPreviousReward && it != m_saveData["reward"].begin(); --it) {
        mean += *it;
    }
    mean /= static_cast<double>(nbPreviousReward);
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
    int nbPreviousTD = static_cast<int>(m_networkConf.getActionRate()) / m_timeStep;
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

void NetworkHandle::learningDecay(size_t iteration) {
    double decay = 1 + getNetworkConfig().getDecayRate() * static_cast<double>(iteration);

//    m_spinet.getNeuron(0, 2).get().learningDecay(decay); // changing m_conf instance reference
//    m_spinet.getNeuron(0, 3).get().learningDecay(decay);

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

/* Opens an event file (in the npz format) and load all the events in memory into a vector.
 * If nbPass is greater than 1, the events are concatenated multiple times and the timestamps are updated in accordance.
 * Returns the vector of events.
 * Only for loadNpzEvents camera event files.
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

bool NetworkHandle::loadHDF5Events(std::vector<Event> &events) {
    if (m_eventFile.offset + m_eventFile.packetSize > m_eventFile.dims) {
        return true;
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

    for (int i = 0; i < m_eventFile.packetSize; i++) {
        events.emplace_back(vT[i], vX[i], vY[i], vP[i], vC[i]);
    }

    m_eventFile.offset += m_eventFile.packetSize; // TODO: do not cut the last events
    return false;
}