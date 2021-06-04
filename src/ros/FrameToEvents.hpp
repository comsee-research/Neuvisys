//
// Created by thomas on 04/06/2021.
//

#ifndef NEUVISYS_FRAMETOEVENTS_HPP
#define NEUVISYS_FRAMETOEVENTS_HPP

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "../network/Event.hpp"

class FrameToEvents {
    int n_max;
    int blocksize;
    float map_threshold;
    int log_threshold;
    float adapt_thresh_coef_shift;
    int method;
    double prevTime;

public:
    void frameConversion(int count, const std::string &topic, const ros::MessageEvent<const sensor_msgs::Image> &frame, cv::Mat &reference, cv::Mat &input, cv::Mat &thresholdmap, cv::Mat &eim, std::vector<Event> &events, int camera);
    explicit FrameToEvents(int n_max=5, int blocksize=1, int log_threshold=20, float map_threshold=0.4, float adapt_thresh_coef_shift=0.05, int method=3);
    void logFrame(cv::Mat &frame) const;
    void convertFrameToEvent(const cv::Mat &input, cv::Mat &reference, cv::Mat &thresholdmap, std::vector<Event> &events, long time, int camera) const;
    int write_event(std::vector<Event> &events, float delta_B, float threshold, long time, int x, int y, int polarity, int camera) const;
    static cv::Mat eventImage(const cv::Size &size, const std::vector<Event> &events);
};


#endif //NEUVISYS_FRAMETOEVENTS_HPP
