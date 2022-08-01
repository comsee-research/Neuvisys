//
// Created by Thomas on 04/06/2021.
//

#ifndef NEUVISYS_SIMULATIONINTERFACE_HPP
#define NEUVISYS_SIMULATIONINTERFACE_HPP

//std
#include <sstream>

//neuvisys
#include <network/NetworkHandle.hpp>

//simulator
#include "Motor.hpp"
#include "FrameToEvents.hpp"

class SimulationInterface {
    ros::NodeHandle nh;
    ros::Subscriber m_leftSensorSub;
    ros::Subscriber m_rightSensorSub;
    ros::Subscriber m_leftMotorAngleSub;
    ros::Subscriber m_timeSub;
    ros::Subscriber m_simStepDoneSub;
    ros::Subscriber m_simTimeStepSub;
    ros::Publisher m_startSimulation;
    ros::Publisher m_stopSimulation;
    ros::Publisher m_enableSyncMode;
    ros::Publisher m_triggerNextStep;
    Motor m_leftMotorLeftRightPub = Motor(nh, "leftmotorleftright");
    Motor m_leftMotorUpDownPub = Motor(nh, "leftmotorupdown");
    Motor m_leftMotorRotationPub = Motor(nh, "leftmotorrotation");
    ros::Time m_lastImageTime, m_imageTime;
    double m_elapsedTime{};
    double m_lambda{};
    double m_time{};
    double m_timeStep{};
    bool m_simStepDone = false;
    bool m_saveEvents = false;

    size_t count = 0;

    double m_rewardStored{};
    FrameToEvents frameConverter;
    cv::Mat leftReference, leftThresholdmap, leftEim;
    cv::Mat rightReference, rightThresholdmap, rightEim;
    bool firstLeftImage = true, firstRightImage = true;
    std::vector<Event> leftEvents, rightEvents;
    std::vector<std::pair<uint64_t, float>> m_actionMapping;

public:
    explicit SimulationInterface(const std::vector<std::pair<uint64_t, float>>& actions, bool saveFrames=false, bool saveEvents=false);
    void update();
    const std::vector<Event> &getLeftEvents() { return leftEvents; }
    const std::vector<Event> &getRightEvents() { return rightEvents; }
    [[nodiscard]] double getReward() const { return m_rewardStored; }
    [[nodiscard]] bool simStepDone() const { return m_simStepDone; }
    [[nodiscard]] double getSimulationTime() const { return m_time; }
    [[nodiscard]] double getSimulationTimeStep() const { return m_timeStep; }
    void motorsJitter(double dt);
    void activateMotors(uint64_t action);
    void enableSyncMode(bool enable);
    void triggerNextTimeStep();
    void startSimulation();
    void stopSimulation();

private:
    void visionCallBack(const ros::MessageEvent<const sensor_msgs::Image> &frame, const std::string &topic);
    void timeCallBack(const ros::MessageEvent<std_msgs::Float32> &time);
    void timeStepCallBack(const ros::MessageEvent<std_msgs::Float32> &timeStep);
    void jointAngleCallBack(const ros::MessageEvent<std_msgs::Float32> &jointAngle);
    void simulationStepDoneCallBack(const ros::MessageEvent<std_msgs::Bool> &simStepDone);
    bool poissonProcess();
};

#endif //NEUVISYS_SIMULATIONINTERFACE_HPP
