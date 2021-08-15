//
// Created by alphat on 14/04/2021.
//

#include "SimulationInterface.hpp"

SimulationInterface::SimulationInterface(double lambda) : m_lambda(lambda) {
    m_rewardSub = nh.subscribe<std_msgs::Float32>("reward", 1000, [this](auto && PH1) { rewardSignal(std::forward<decltype(PH1)>(PH1)); });
    m_leftSensorSub = nh.subscribe<sensor_msgs::Image>("leftimage", 1000,
                                                      [this](auto && PH1) { visionCallBack(std::forward<decltype(PH1)>(PH1), "left"); });
//    m_rightSensorSub = n.subscribe<sensor_msgs::Image>("rightimage", 1000,
//                                                       boost::bind(&SimulationInterface::visionCallBack, this, _1, "right"));

    motorMapping.emplace_back(std::make_pair(0, 0.15)); // left horizontal -> left movement
//    motorMapping.emplace_back(std::make_pair(0, 0)); // left horizontal -> no movement
    motorMapping.emplace_back(std::make_pair(0, -0.15)); // left horizontal  -> right movement
//    motorMapping.emplace_back(std::make_pair(1, -0.1)); // left vertical  -> left movement
//    motorMapping.emplace_back(std::make_pair(1, 0)); // left vertical -> no movement
//    motorMapping.emplace_back(std::make_pair(1, 0.1)); // left vertical -> right movement
}

void SimulationInterface::visionCallBack(const ros::MessageEvent<sensor_msgs::Image const> &frame, const std::string &topic) {
    if (topic == "left") {
        frameConverter.frameConversion(topic, frame, leftReference, leftInput, leftThresholdmap, leftEim, leftEvents, 0);
        receivedLeftImage = true;
    } else if (topic == "right") {
//        rightConverter.frameConversion(topic, frame, rightReference, rightInput, rightThresholdmap, rightEim, rightEvents, 1);
//        receivedRightImage = true;
    } else {
        std::cout << "wrong camera topic" << std::endl;
        return;
    }
    m_imageTime = frame.getMessage()->header.stamp;
}

void SimulationInterface::rewardSignal(const ros::MessageEvent<std_msgs::Float32> &reward) {
    m_rewardStored = reward.getMessage()->data;
}

void SimulationInterface::update() {
    auto dt = (m_imageTime - m_lastImageTime).toSec();
    m_lastImageTime = m_imageTime;

//    motorsJitter(dt);
}

bool SimulationInterface::motorAction(const std::vector<uint64_t> &motorActivation, const double explorationFactor, int &selectedMotor) {
    bool exploration = false;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distReal(0.0, 1.0);
    std::uniform_int_distribution<> distInt(0, static_cast<int>(motorActivation.size() - 1));

    auto real = distReal(gen);
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

void SimulationInterface::activateMotors(std::vector<uint64_t> motorActivation) {
    for (size_t i = 0; i < motorActivation.size(); ++i) {
        if (motorActivation[i] > 0) {
            switch (motorMapping[i].first) {
                case 0:
                    m_leftMotor1Pub.move(motorMapping[i].second);
                    break;
                case 1:
                    m_leftMotor2Pub.move(motorMapping[i].second);
                    break;
                case 2:
                    m_rightMotor1Pub.move(motorMapping[i].second);
                    break;
                case 3:
                    m_rightMotor2Pub.move(motorMapping[i].second);
                    break;
            }
        }
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
    m_leftMotor1Pub.jitter(dt);
    m_leftMotor2Pub.jitter(dt);
    m_rightMotor1Pub.jitter(dt);
    m_rightMotor2Pub.jitter(dt);
}

void SimulationInterface::activateMotor(uint64_t motor) {
    switch (motorMapping[motor].first) {
        case 0:
            m_leftMotor1Pub.move(motorMapping[motor].second);
            break;
        case 1:
            m_leftMotor2Pub.move(motorMapping[motor].second);
            break;
        case 2:
            m_rightMotor1Pub.move(motorMapping[motor].second);
            break;
        case 3:
            m_rightMotor2Pub.move(motorMapping[motor].second);
            break;
    }
}
