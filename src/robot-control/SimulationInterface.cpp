//
// Created by alphat on 14/04/2021.
//

#include "SimulationInterface.hpp"

SimulationInterface::SimulationInterface() {
    m_rewardSub = nh.subscribe<std_msgs::Float32>("reward", 1000, [this](auto && PH1) { rewardSignalCallBack(std::forward<decltype(PH1)>(PH1)); });
    m_leftSensorSub = nh.subscribe<sensor_msgs::Image>("leftimage", 1000,
                                                      [this](auto && PH1) { visionCallBack(std::forward<decltype(PH1)>(PH1), "left"); });
    m_rightSensorSub = nh.subscribe<sensor_msgs::Image>("rightimage", 1000,
                                                       [this](auto && PH1) { visionCallBack(std::forward<decltype(PH1)>(PH1), "right"); });
    m_timeSub = nh.subscribe<std_msgs::Float32>("simulationTime", 1000, [this](auto && PH1) { timeCallBack(std::forward<decltype(PH1)>(PH1)); });
    m_simTimeStepSub = nh.subscribe<std_msgs::Float32>("simulationTimeStep", 1000, [this](auto && PH1) { timeStepCallBack(std::forward<decltype(PH1)>(PH1)); });
    m_simStepDoneSub = nh.subscribe<std_msgs::Bool>("simulationStepDone", 1000, [this](auto && PH1) { simulationStepDoneCallBack(std::forward<decltype(PH1)>(PH1)); });

    m_startSimulation = nh.advertise<std_msgs::Bool>("startSimulation", 1000);
    m_stopSimulation = nh.advertise<std_msgs::Bool>("stopSimulation", 1000);
    m_enableSyncMode = nh.advertise<std_msgs::Bool>("enableSyncMode", 1000);
    m_triggerNextStep = nh.advertise<std_msgs::Bool>("triggerNextStep", 1000);

    while (m_startSimulation.getNumSubscribers() < 1 && m_stopSimulation.getNumSubscribers() < 1 && m_enableSyncMode.getNumSubscribers() < 1 && m_triggerNextStep.getNumSubscribers() < 1) {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    motorMapping.emplace_back(std::make_pair(0, 0.15)); // left horizontal -> left movement
    motorMapping.emplace_back(std::make_pair(0, 0)); // no movement
    motorMapping.emplace_back(std::make_pair(0, -0.15)); // left horizontal  -> right movement

//    motorMapping.emplace_back(std::make_pair(0, 0.05)); // increment speed left
//    motorMapping.emplace_back(std::make_pair(0, -0.05)); // increment speed right
}

void SimulationInterface::visionCallBack(const ros::MessageEvent<sensor_msgs::Image const> &frame, const std::string &topic) {
    if (topic == "left") {
        leftEvents.clear();
        frameConverter.frameConversion(topic, frame, leftReference, leftThresholdmap, leftEim, leftEvents, 0);
    } else if (topic == "right") {
//        rightEvents.clear();
//        frameConverter.frameConversion(topic, frame, rightReference, rightThresholdmap, rightEim, rightEvents, 1);
    } else {
        std::cout << "wrong camera topic" << std::endl;
        return;
    }
    m_imageTime = frame.getMessage()->header.stamp;
}

void SimulationInterface::rewardSignalCallBack(const ros::MessageEvent<std_msgs::Float32> &reward) {
    m_rewardStored = reward.getMessage()->data;
}

void SimulationInterface::timeCallBack(const ros::MessageEvent<std_msgs::Float32> &time) {
    m_time = time.getMessage()->data;
}

void SimulationInterface::timeStepCallBack(const ros::MessageEvent<std_msgs::Float32> &timeStep) {
    m_timeStep = timeStep.getMessage()->data;
}

void SimulationInterface::simulationStepDoneCallBack(const ros::MessageEvent<std_msgs::Bool> &simStepDone) {
    m_simStepDone = simStepDone.getMessage()->data;
}

void SimulationInterface::update() {
    m_simStepDone = false;
    auto dt = (m_imageTime - m_lastImageTime).toSec();
    m_lastImageTime = m_imageTime;

    motorsJitter(dt);
}

bool SimulationInterface::motorAction(const std::vector<uint64_t> &motorActivation, const double explorationFactor, int &selectedMotor) {
    bool exploration = false;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distReal(0.0, 1.0);
    std::uniform_int_distribution<> distInt(0, static_cast<int>(motorActivation.size() - 1));

    auto real = 100 * distReal(gen);
    if (real >= explorationFactor) {
        selectedMotor = Util::winnerTakeAll(motorActivation);
    } else {
        selectedMotor = distInt(gen);
        exploration = true;
    }

    if (selectedMotor != -1) {
        activateMotor(selectedMotor);
    }
    return exploration;
}

bool SimulationInterface::poissonProcess() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    double random = dist(gen);
    double F = 1 - std::exp(- m_lambda * m_elapsedTime);

    if (random < F) {
        m_elapsedTime = 0;
        return true;
    }
    return false;
}

void SimulationInterface::motorsJitter(double dt) {
    m_leftMotor1Pub.jitter(dt);
    m_leftMotor2Pub.jitter(dt, m_leftMotor1Pub.getJitterPos());

    m_rightMotor1Pub.jitter(dt);
    m_rightMotor2Pub.jitter(dt, m_leftMotor2Pub.getJitterPos());
}

void SimulationInterface::activateMotor(uint64_t motor) {
    switch (motorMapping[motor].first) {
        case 0:
            m_leftMotor1Pub.moveSpeed(motorMapping[motor].second);
            break;
        case 1:
            m_leftMotor2Pub.moveSpeed(motorMapping[motor].second);
            break;
        case 2:
            m_rightMotor1Pub.moveSpeed(motorMapping[motor].second);
            break;
        case 3:
            m_rightMotor2Pub.moveSpeed(motorMapping[motor].second);
            break;
    }
}

void SimulationInterface::startSimulation() {
    std_msgs::Bool msg{};
    msg.data = true;
    m_startSimulation.publish(msg);
}

void SimulationInterface::stopSimulation() {
    std_msgs::Bool msg{};
    msg.data = true;
    m_stopSimulation.publish(msg);
}

void SimulationInterface::enableSyncMode(bool enable) {
    std_msgs::Bool msg{};
    msg.data = enable;
    m_enableSyncMode.publish(msg);
}

void SimulationInterface::triggerNextTimeStep() {
    std_msgs::Bool msg{};
    msg.data = true;
    m_triggerNextStep.publish(msg);
}
