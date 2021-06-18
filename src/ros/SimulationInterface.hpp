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
    int m_leftCount, m_rightCount;
    ros::Time m_imageTime;

    ros::NodeHandle n;
    Motor m_leftMotor1Pub = Motor(n, "leftmotor1");
    Motor m_leftMotor2Pub = Motor(n, "leftmotor2");
    Motor m_rightMotor1Pub = Motor(n, "rightmotor1");
    Motor m_rightMotor2Pub = Motor(n, "rightmotor2");
    ros::Subscriber m_leftSensorSub;
    ros::Subscriber m_rightSensorSub;
    ros::Subscriber m_rewardSub;

    double m_rewardStored;

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
    ~SimulationInterface();
    void motorCommands(double dt);
    void visionCallBack(const ros::MessageEvent<const sensor_msgs::Image> &frame, const std::string &topic);
    void rewardSignal(const ros::MessageEvent<std_msgs::Float32> &reward);
};

#endif //NEUVISYS_SIMULATIONINTERFACE_HPP
