//
// Created by thomas on 04/06/2021.
//

#ifndef NEUVISYS_SIMULATIONINTERFACE_HPP
#define NEUVISYS_SIMULATIONINTERFACE_HPP

#include <sstream>
#include "Motor.hpp"
#include "../network/SpikingNetwork.hpp"
#include "FrameToEvents.hpp"

class SimulationInterface {
    int leftCount, rightCount;
    ros::Time imageTime;

    ros::NodeHandle n;
    Motor leftMotor1Pub = Motor(n, "leftmotor1");
    Motor leftMotor2Pub = Motor(n, "leftmotor2");
    Motor rightMotor1Pub = Motor(n, "rightmotor1");
    Motor rightMotor2Pub = Motor(n, "rightmotor2");
    ros::Subscriber leftSensor;
    ros::Subscriber rightSensor;

    FrameToEvents converter = FrameToEvents(5, 1, 1, 0.2, 0, 3);
    cv::Mat leftReference, leftInput, leftThresholdmap, leftEim;
    cv::Mat rightReference, rightInput, rightThresholdmap, rightEim;
    std::vector<Event> leftEvents, rightEvents;

    std::vector<std::pair<uint64_t, float>> motorMapping;
    std::vector<bool> motorActivation;
    std::string networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
    NetworkConfig config = NetworkConfig(networkPath);
    SpikingNetwork spinet = SpikingNetwork(config);

public:
    SimulationInterface();
    void motorCommands(double dt);
    void visionCallBack(const ros::MessageEvent<const sensor_msgs::Image> &frame, const std::string &topic);
};

#endif //NEUVISYS_SIMULATIONINTERFACE_HPP
