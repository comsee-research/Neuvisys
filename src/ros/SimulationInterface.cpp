//
// Created by alphat on 14/04/2021.
//

#include "SimulationInterface.hpp"

SimulationInterface::SimulationInterface() {
    m_rewardSub = nh.subscribe<std_msgs::Float32>("reward", 1000, [this](auto && PH1) { rewardSignal(std::forward<decltype(PH1)>(PH1)); });
    m_leftSensorSub = nh.subscribe<sensor_msgs::Image>("leftimage", 1000,
                                                      [this](auto && PH1) { visionCallBack(std::forward<decltype(PH1)>(PH1), "left"); });
//    m_rightSensorSub = n.subscribe<sensor_msgs::Image>("rightimage", 1000,
//                                                       boost::bind(&SimulationInterface::visionCallBack, this, _1, "right"));

    motorMapping.emplace_back(std::make_pair(0, -0.02)); // left horizontal -> left movement
    motorMapping.emplace_back(std::make_pair(0, 0)); // left horizontal -> no movement
    motorMapping.emplace_back(std::make_pair(0, 0.02)); // left horizontal  -> right movement
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

double SimulationInterface::update() {
    auto dt = (m_imageTime - m_lastImageTime).toSec();

//    if (hasReceivedLeftImage()) {
//        auto motorActivation = m_spinet.run(leftEvents, m_rewardStored);
//        activateMotors(motorActivation, dt);
//        resetLeft();
//    }
//    if (hasReceivedRightImage()) {
//        auto motorActivation = m_spinet.run(rightEvents, m_rewardStored);
//        activateMotors(motorActivation, dt);
//        resetRight();
//    }
    m_lastImageTime = m_imageTime;
    return dt;
}

void SimulationInterface::activateMotors(std::vector<uint64_t> motorActivation) {
    for (size_t i = 0; i < motorActivation.size(); ++i) {
        if (motorActivation[i] > 0) {
            switch (motorMapping[i].first) {
                case 0:
                    m_leftMotor1Pub.move(motorMapping[i].second);
                    break;
                    //                case 1:
                    //                    m_leftMotor2Pub.move(motorMapping[i].second);
                    //                    break;
                    //                case 2:
                    //                    m_rightMotor1Pub.move(motorMapping[i].second);
                    //                    break;
                    //                case 3:
                    //                    m_rightMotor2Pub.move(motorMapping[i].second);
                    //                    break;
            }
        }
    }

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
            //        case 1:
            //            m_leftMotor2Pub.move(motorMapping[motor].second);
            //            break;
            //        case 2:
            //            m_rightMotor1Pub.move(motorMapping[motor].second);
            //            break;
            //        case 3:
            //            m_rightMotor2Pub.move(motorMapping[motor].second);
            //            break;
    }
}
