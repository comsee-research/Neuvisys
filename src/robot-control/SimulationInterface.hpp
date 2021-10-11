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
    ros::Subscriber m_timeSub;
    ros::Subscriber m_simStepDoneSub;
    ros::Publisher m_startSimulation;
    ros::Publisher m_stopSimulation;
    ros::Publisher m_enableSyncMode;
    ros::Publisher m_triggerNextStep;
    Motor m_leftMotor1Pub = Motor(nh, "leftmotor1");
    Motor m_leftMotor2Pub = Motor(nh, "leftmotor2");
    Motor m_rightMotor1Pub = Motor(nh, "rightmotor1");
    Motor m_rightMotor2Pub = Motor(nh, "rightmotor2");
    ros::Time m_lastImageTime, m_imageTime;
    double m_elapsedTime{};
    double m_lambda{};
    double m_time{};
    bool m_simStepDone = false;

    double m_rewardStored{};
    FrameToEvents frameConverter = FrameToEvents(5, 1, 1, 0.2, 0, 3);
    cv::Mat leftReference, leftThresholdmap, leftEim;
    cv::Mat rightReference, rightThresholdmap, rightEim;
    std::vector<Event> leftEvents, rightEvents;
    std::vector<std::pair<uint64_t, float>> motorMapping;

public:
    explicit SimulationInterface(double lambda);
    void update();
    bool motorAction(const std::vector<uint64_t> &motorActivation, double explorationFactor, int &selectedMotor);
    const std::vector<Event> &getLeftEvents() { return leftEvents; }
    const std::vector<Event> &getRightEvents() { return rightEvents; }
    [[nodiscard]] double getReward() const { return m_rewardStored; }
    [[nodiscard]] bool simStepDone() const { return m_simStepDone; }
    [[nodiscard]] double getSimulationTime() const { return m_time; }
    void activateMotors(std::vector<uint64_t> motorActivation);
    void motorsJitter(double dt);
    void activateMotor(uint64_t motor);
    void enableSyncMode(bool enable);
    void triggerNextTimeStep();
    void startSimulation();
    void stopSimulation();

private:
    void visionCallBack(const ros::MessageEvent<const sensor_msgs::Image> &frame, const std::string &topic);
    void timeCallBack(const ros::MessageEvent<std_msgs::Float32> &time);
    void rewardSignal(const ros::MessageEvent<std_msgs::Float32> &reward);
    void simulationStepDoneCallBack(const ros::MessageEvent<std_msgs::Bool> &simStepDone);
    bool poissonProcess();
};

#endif //NEUVISYS_SIMULATIONINTERFACE_HPP
