//
// Created by Thomas on 14/04/2021.
//

#include "SimulationInterface.hpp"

SimulationInterface::SimulationInterface(const std::vector<std::pair<uint64_t, float>> &actions, bool saveFrames, bool saveEvents) : m_saveEvents(saveEvents) {
    frameConverter = FrameToEvents(5, 1, 0, 0.4, 0, 1, saveFrames, m_saveEvents);
    m_leftSensorSub = nh.subscribe<sensor_msgs::Image>("leftimage", 1000,
                                                      [this](auto && PH1) { visionCallBack(std::forward<decltype(PH1)>(PH1), "left"); });
    m_rightSensorSub = nh.subscribe<sensor_msgs::Image>("rightimage", 1000,
                                                       [this](auto && PH1) { visionCallBack(std::forward<decltype(PH1)>(PH1), "right"); });
    m_leftMotorAngleSub = nh.subscribe<std_msgs::Float32>("jointangle", 1000,
                                                          [this](auto && PH1) { jointAngleCallBack(std::forward<decltype(PH1)>(PH1)); });
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

    for (auto pair : actions) {
        m_actionMapping.emplace_back(pair);
    }
}

void SimulationInterface::visionCallBack(const ros::MessageEvent<sensor_msgs::Image const> &frame, const std::string &topic) {
    if (topic == "left") {
        leftEvents.clear();
        frameConverter.frameConversion(topic, frame, leftReference, leftThresholdmap, leftEim, leftEvents, 0, firstLeftImage);
        if (firstLeftImage) {
            firstLeftImage = false;
        }
    } else if (topic == "right") {
        rightEvents.clear();
        frameConverter.frameConversion(topic, frame, rightReference, rightThresholdmap, rightEim, rightEvents, 1, firstRightImage);
        if (firstRightImage) {
            firstRightImage = false;
        }
    } else {
        std::cout << "wrong camera topic" << std::endl;
        return;
    }
    m_imageTime = frame.getMessage()->header.stamp;
}

void SimulationInterface::jointAngleCallBack(const ros::MessageEvent<std_msgs::Float32> &jointAngle) {
    m_rewardStored = jointAngle.getMessage()->data; // Tracking
//    m_rewardStored = 50 * std::abs(std::abs(jointAngle.getMessage()->data) - (M_PI / 2));
//    m_rewardStored = 50 * ((M_PI / 2) - std::abs(std::abs(jointAngle.getMessage()->data) - (M_PI / 2)));
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

    if (dt != 0) {
//        motorsJitter(dt);
    }
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
    m_leftMotorLeftRightPub.jitter(dt);
    m_leftMotorUpDownPub.jitter(dt);
}

void SimulationInterface::activateMotors(uint64_t action) {
    auto pair = m_actionMapping[action];
    if (pair.first == 1) {
        m_leftMotorPhiPub.changeSpeed(pair.second);
        m_leftMotorThetaPub.changeSpeed(0);
    } else if (pair.first == 2) {
        m_leftMotorThetaPub.changeSpeed(pair.second);
        m_leftMotorPhiPub.changeSpeed(0);
    }
}

void SimulationInterface::startSimulation() {
    std_msgs::Bool msg{};
    msg.data = true;
    m_startSimulation.publish(msg);
    std::cout << "Starting simulation" << std::endl;
}

void SimulationInterface::stopSimulation() {
    std_msgs::Bool msg{};
    msg.data = true;
    m_stopSimulation.publish(msg);
    std::cout << "Stopping simulation" << std::endl;

    if (m_saveEvents) {
        frameConverter.saveEventsAsFile("/home/thomas/Desktop/events");
    }
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
