//
// Created by alphat on 14/04/2021.
//

#include "SimulationInterface.hpp"

SimulationInterface::SimulationInterface() {
    leftCount = 0; rightCount = 0;

    reward = n.subscribe<std_msgs::Float32>("reward", 1000, boost::bind(&SimulationInterface::rewardSignal, this, _1));
    leftSensor = n.subscribe<sensor_msgs::Image>("leftimage", 1000,
                                    boost::bind(&SimulationInterface::visionCallBack, this, _1, "left"));
    rightSensor = n.subscribe<sensor_msgs::Image>("rightimage", 1000,
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
        converter.frameConversion(leftCount, topic, frame, leftReference, leftInput, leftThresholdmap, leftEim, leftEvents, 0);
        motorActivation = spinet.run(leftEvents, rewardStored);
        leftEvents.clear();
        leftCount++;
    } else if (topic == "right") {
        converter.frameConversion(rightCount, topic, frame, rightReference, rightInput, rightThresholdmap, rightEim, rightEvents, 1);
        motorActivation = spinet.run(rightEvents, rewardStored);
        rightEvents.clear();
        rightCount++;
    } else {
        std::cout << "wrong camera topic" << std::endl;
        return;
    }
    auto dt = (frame.getMessage()->header.stamp - imageTime).toSec();
    if (dt > 0) {
        motorCommands(dt);
    }
    imageTime = frame.getMessage()->header.stamp;
}

void SimulationInterface::motorCommands(double dt) {
//    leftMotor1Pub.jitter(dt);
//    leftMotor2Pub.jitter(dt);
//    rightMotor1Pub.jitter(dt);
//    rightMotor2Pub.jitter(dt);

    for (size_t i = 0; i < motorActivation.size(); ++i) {
        if (motorActivation[i]) {
            switch (motorMapping[i].first) {
                case 0:
                    leftMotor1Pub.move(motorMapping[i].second);
                    break;
                case 1:
                    leftMotor2Pub.move(motorMapping[i].second);
                    break;
                case 2:
                    rightMotor1Pub.move(motorMapping[i].second);
                    break;
                case 3:
                    rightMotor2Pub.move(motorMapping[i].second);
                    break;
            }
        }
    }
}

void SimulationInterface::rewardSignal(const ros::MessageEvent<std_msgs::Float32> reward) {
    rewardStored = reward.getMessage()->data;
//    std::cout << rewardStored << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    SimulationInterface sim;
    ros::spin();
    return 0;
}
