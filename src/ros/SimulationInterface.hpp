//
// Created by thomas on 04/06/2021.
//

#ifndef NEUVISYS_SIMULATIONINTERFACE_HPP
#define NEUVISYS_SIMULATIONINTERFACE_HPP

#include <sstream>

#include "../network/SpikingNetwork.hpp"
#include "Motor.hpp"
#include "FrameToEvents.hpp"

class SimulationInterface {
    ros::NodeHandle nh;
    ros::Subscriber m_leftSensorSub;
    ros::Subscriber m_rightSensorSub;
    ros::Subscriber m_rewardSub;
    Motor m_leftMotor1Pub = Motor(nh, "leftmotor1");
    Motor m_leftMotor2Pub = Motor(nh, "leftmotor2");
    Motor m_rightMotor1Pub = Motor(nh, "rightmotor1");
    Motor m_rightMotor2Pub = Motor(nh, "rightmotor2");
    ros::Time m_lastImageTime, m_imageTime;

    double m_rewardStored{};
    long et = 0;
    bool receivedLeftImage = false, receivedRightImage = false;
    FrameToEvents frameConverter = FrameToEvents(5, 1, 1, 0.2, 0, 3);
    cv::Mat leftReference, leftInput, leftThresholdmap, leftEim;
    cv::Mat rightReference, rightInput, rightThresholdmap, rightEim;
    std::vector<Event> leftEvents, rightEvents;
    std::vector<std::pair<uint64_t, float>> motorMapping;

public:
    SimulationInterface();
    double update();
    const std::vector<Event> &getLeftEvents() { return leftEvents; }
    const std::vector<Event> &getRightEvents() { return rightEvents; }
    void resetLeft() { leftEvents.clear(); receivedLeftImage = false; }
    void resetRight() { rightEvents.clear(); receivedRightImage = false; }
    double getReward() const { return m_rewardStored; }
    bool hasReceivedLeftImage() const { return receivedLeftImage; }
    bool hasReceivedRightImage() const { return receivedRightImage; }
    void activateMotors(uint64_t motor, double dt);

private:
    void visionCallBack(const ros::MessageEvent<const sensor_msgs::Image> &frame, const std::string &topic);
    void rewardSignal(const ros::MessageEvent<std_msgs::Float32> &reward);
};

#endif //NEUVISYS_SIMULATIONINTERFACE_HPP
