//
// Created by alphat on 14/04/2021.
//

#include "SimulationInterface.hpp"

SimulationInterface::SimulationInterface() {
    m_leftCount = 0; m_rightCount = 0;

    m_rewardSub = n.subscribe<std_msgs::Float32>("m_rewardSub", 1000, boost::bind(&SimulationInterface::rewardSignal, this, _1));
    m_leftSensorSub = n.subscribe<sensor_msgs::Image>("leftimage", 1000,
                                                      boost::bind(&SimulationInterface::visionCallBack, this, _1, "left"));
    m_rightSensorSub = n.subscribe<sensor_msgs::Image>("rightimage", 1000,
                                                       boost::bind(&SimulationInterface::visionCallBack, this, _1, "right"));

    motorMapping.emplace_back(std::make_pair(0, -0.1));
    motorMapping.emplace_back(std::make_pair(0, 0));
    motorMapping.emplace_back(std::make_pair(0, 0.1));
    motorMapping.emplace_back(std::make_pair(1, -0.1));
    motorMapping.emplace_back(std::make_pair(1, 0));
    motorMapping.emplace_back(std::make_pair(1, 0.1));
}

void SimulationInterface::visionCallBack(const ros::MessageEvent<sensor_msgs::Image const> &frame, const std::string &topic) {
    if (topic == "left") {
        converter.frameConversion(m_leftCount, topic, frame, leftReference, leftInput, leftThresholdmap, leftEim, leftEvents, 0);
//        motorActivation = spinet.run(leftEvents, m_rewardStored);
        leftEvents.clear();
        m_leftCount++;
    } else if (topic == "right") {
        converter.frameConversion(m_rightCount, topic, frame, rightReference, rightInput, rightThresholdmap, rightEim, rightEvents, 1);
//        motorActivation = spinet.run(rightEvents, m_rewardStored);
        rightEvents.clear();
        m_rightCount++;
    } else {
        std::cout << "wrong camera topic" << std::endl;
        return;
    }
    auto dt = (frame.getMessage()->header.stamp - m_imageTime).toSec();
    if (dt > 0) {
//        motorCommands(dt);
    }
    m_imageTime = frame.getMessage()->header.stamp;
}

void SimulationInterface::motorCommands(double dt) {
//    m_leftMotor1Pub.jitter(dt);
//    m_leftMotor2Pub.jitter(dt);
//    m_rightMotor1Pub.jitter(dt);
//    m_rightMotor2Pub.jitter(dt);

    for (size_t i = 0; i < motorActivation.size(); ++i) {
        if (motorActivation[i]) {
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

void SimulationInterface::rewardSignal(const ros::MessageEvent<std_msgs::Float32> &reward) {
    m_rewardStored = reward.getMessage()->data;
//    std::cout << m_rewardStored << std::endl;
}

SimulationInterface::~SimulationInterface() {
//    spinet.saveNetworkLearningTrace(1, "ros");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    SimulationInterface sim;

    ros::spin();

//    auto start = std::chrono::system_clock::now();
//    auto time = std::chrono::system_clock::now();
//    while(std::chrono::duration_cast<std::chrono::milliseconds>(time - start).count() < 5000) {
//        time = std::chrono::system_clock::now();
//        ros::spinOnce();
//    }

    return 0;
}
