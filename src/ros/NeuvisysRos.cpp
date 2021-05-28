//
// Created by alphat on 14/04/2021.
//

#include <sstream>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../network/SpikingNetwork.hpp"

class SimulationInterface {

private:
    int time_gap;
    int n_max;
    int blocksize;
    float map_threshold;
    int log_threshold;
    float adapt_thresh_coef_shift;
    int leftCount, rightCount;
    int method;

    std_msgs::Float32 position;

	ros::NodeHandle n;
	ros::Subscriber leftImageSub, rightImageSub;
    ros::Publisher leftMotor1Pub, leftMotor2Pub, rightMotor1Pub, rightMotor2Pub;

	cv::Mat leftReference, leftInput, leftThresholdmap, leftEim;
    cv::Mat rightReference, rightInput, rightThresholdmap, rightEim;
	std::vector<Event> leftEvents, rightEvents;

	std::map<uint64_t, std::vector<std::pair<uint64_t, float>>> motorMapping;
    std::vector<bool> motorActivation;

    std::string networkPath = "/home/thomas/neuvisys-dv/configuration/network/configs/network_config.json";
	NetworkConfig config = NetworkConfig(networkPath);
	SpikingNetwork spinet = SpikingNetwork(config);

public:
    SimulationInterface(float time_gap, int n_max=5, int blocksize=1, int log_threshold=20, float map_threshold=0.4, float adapt_thresh_coef_shift=0.05, int method=3) : time_gap(time_gap), n_max(n_max), blocksize(blocksize), map_threshold(map_threshold), log_threshold(log_threshold), adapt_thresh_coef_shift(adapt_thresh_coef_shift), method(method) {
        leftCount = 0; rightCount = 0;

        leftMotor1Pub = n.advertise<std_msgs::Float32>("leftmotor1", 1000); // 0
        leftMotor2Pub = n.advertise<std_msgs::Float32>("leftmotor2", 1000); // 1
        rightMotor1Pub = n.advertise<std_msgs::Float32>("rightmotor1", 1000); // 3
        rightMotor2Pub = n.advertise<std_msgs::Float32>("rightmotor2", 1000); // 4

        motorMapping[0] = {std::make_pair(0, -10), std::make_pair(1, -10)};
        motorMapping[1] = {std::make_pair(0, 0), std::make_pair(1, -10)};
        motorMapping[2] = {std::make_pair(0, 10), std::make_pair(1, -10)};
        motorMapping[3] = {std::make_pair(0, -10), std::make_pair(1, 0)};
        motorMapping[4] = {std::make_pair(0, 0), std::make_pair(1, 0)};
        motorMapping[5] = {std::make_pair(0, 10), std::make_pair(1, 0)};
        motorMapping[6] = {std::make_pair(0, -10), std::make_pair(1, 10)};
        motorMapping[7] = {std::make_pair(0, 0), std::make_pair(1, 10)};
        motorMapping[8] = {std::make_pair(0, 10), std::make_pair(1, 10)};

		leftImageSub = n.subscribe<sensor_msgs::Image>("leftimage", 1000, boost::bind(&SimulationInterface::chatterCallback, this, _1, "left"));
        rightImageSub = n.subscribe<sensor_msgs::Image>("rightimage", 1000, boost::bind(&SimulationInterface::chatterCallback, this, _1, "right"));
    }

	void chatterCallback(const ros::MessageEvent<sensor_msgs::Image const> &frame, const std::string &topic) {
        if (topic == "left") {
            frameConversion(leftCount, topic, frame, leftReference, leftInput, leftThresholdmap, leftEim, leftEvents, 0);
            motorActivation = spinet.run(leftEvents);
            leftEvents.clear();
            leftCount++;
        } else if (topic == "right") {
            frameConversion(rightCount, topic, frame, rightReference, rightInput, rightThresholdmap, rightEim, rightEvents, 1);
            motorActivation = spinet.run(rightEvents);
            rightEvents.clear();
            rightCount++;
        } else {
            std::cout << "wrong camera topic" << std::endl;
            return;
        }
        motorCommands();
	}

private:
	void frameConversion(int count, const std::string &topic, const ros::MessageEvent<sensor_msgs::Image const> &frame, cv::Mat &reference, cv::Mat &input, cv::Mat &thresholdmap, cv::Mat &eim, std::vector<Event> &events, int camera) {
        try {
            cv::cvtColor(cv_bridge::toCvCopy(frame.getMessage(), sensor_msgs::image_encodings::BGR8)->image, input, CV_BGR2GRAY);
            input.convertTo(input, CV_32F);
            logFrame(input);
            if (count == 0) {
                thresholdmap = cv::Mat(input.size(), CV_32F);
                thresholdmap = map_threshold;
                reference = input.clone();
            } else {
                convertFrameToEvent(input, reference, thresholdmap, events, count, camera);

                if (!events.empty()) {
                    eim = eventImage(input.size(), events);

                    double angle = 180;
                    cv::Point2f center((eim.cols - 1) / 2.0, (eim.rows - 1) / 2.0);
                    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);

                    cv::Mat dst;
                    cv::warpAffine(eim, dst, rot, cv::Size(eim.cols, eim.rows));

                    cv::Mat dst2;
                    cv::flip(dst, dst2, 1);

                    cv::imshow(topic, dst2);
                    cv::waitKey(3);
                }
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

	void motorCommands() {
        jitter();
        for (size_t i = 0; i < motorActivation.size(); ++i) {
            if (motorActivation[i]) {
                for (auto mapping : motorMapping[i]) {
                    position.data = position.data + mapping.second;
                    switch (mapping.first) {
                        case 0:
                            leftMotor1Pub.publish(position);
                            break;
                        case 1:
                            leftMotor2Pub.publish(position);
                            break;
                        case 2:
                            rightMotor1Pub.publish(position);
                            break;
                        case 3:
                            rightMotor2Pub.publish(position);
                            break;
                    }
                }
            }
        }
    }

    void jitter() {
        position.data = 0 + 0.0005 * (rand() % 20 - 10);
        leftMotor1Pub.publish(position);
        position.data = 0 + 0.0005 * (rand() % 20 - 10);
        leftMotor2Pub.publish(position);

        position.data = 0 + 0.0005 * (rand() % 20 - 10);
        rightMotor1Pub.publish(position);
        position.data = 0 + 0.0005 * (rand() % 20 - 10);
        rightMotor2Pub.publish(position);
    }

	cv::Mat eventImage(const cv::Size& size, const std::vector<Event>& events) {
        cv::Mat eim = cv::Mat::zeros(size, CV_8UC3);
        cv::Vec3b color;
        for (Event event: events) {
            if (event.polarity() == 0) {
                color = cv::Vec3b(0, 255, 0);
            } else {
                color = cv::Vec3b(0, 0, 255);
            }
            eim.at<cv::Vec3b>(cv::Point(event.y(), event.x())) = color;
        }
        return eim;
    }

    void logFrame(cv::Mat &frame) const {
        for (int i = 0; i < frame.rows; i++) {
            for (int j = 0; j < frame.cols; j++) {
                if (frame.at<float>(i, j) > log_threshold) {
                    frame.at<float>(i, j) = log(frame.at<float>(i, j));
                }
            }
        }
    }

    int write_event(std::vector<Event> &events, float delta_B, float threshold, int frame_id, int x, int y, int polarity, int camera) const {
        int moddiff = delta_B / threshold;
        int evenum;

        if (moddiff > n_max) {
            evenum = n_max;
        } else {
            evenum = moddiff;
        }

        for (int e = 0; e < evenum; e++) {
            double timestamp = ((time_gap * (e + 1) * threshold) / delta_B) + time_gap * frame_id;
            events.emplace_back(timestamp, x, y, polarity, camera);
        }

        return evenum;
    }

    void convertFrameToEvent(const cv::Mat& input, cv::Mat &reference, cv::Mat &thresholdmap, std::vector<Event> &events, int frame_id, int camera) {
        for (int i = 0; i < input.rows; i = i + blocksize) {
            for (int j = 0; j < input.cols; j = j + blocksize) {
                // Find local maxima --
                // Set blocksize to 0 to disable Local Inhibition
                float diff;
                float diff_ = 0;
                int i_shift = 0;
                int j_shift = 0;
                for (int ii = 0; ii < blocksize; ii++) {
                    for (int jj = 0; jj < blocksize; jj++) {
                        diff = abs(input.at<float>(i + ii, j + jj) - reference.at<float>(i + ii, j + jj));
                        if (diff > diff_) {
                            diff_ = diff;
                            i_shift = ii;
                            j_shift = jj;
                        }
                    }
                }

                // delta_B: Brightness Difference
                float delta_B = input.at<float>(i + i_shift, j + j_shift) - reference.at<float>(i + i_shift, j + j_shift);

                if (delta_B > thresholdmap.at<float>(i + i_shift, j + j_shift)) {
                    int event_num = write_event(events, delta_B, thresholdmap.at<float>(i + i_shift, j + j_shift), frame_id, i + i_shift, j + j_shift, 1, camera);

                    // Update reference
                    if (method == 2) {
                        reference.at<float>(i, j) = input.at<float>(i, j);
                    }
                    if (method == 3) {
                        reference.at<float>(i, j) = reference.at<float>(i, j) + event_num * thresholdmap.at<float>(i, j);
                    }

                    // Update threshold map (Increase)
                    thresholdmap.at<float>(i, j) = (1 + adapt_thresh_coef_shift) * thresholdmap.at<float>(i, j);
                }

                else if (delta_B < -thresholdmap.at<float>(i + i_shift, j + j_shift)) {
                    delta_B = -delta_B;
                    int event_num = write_event(events, delta_B, thresholdmap.at<float>(i + i_shift, j + j_shift), frame_id, i + i_shift, j + j_shift, 0, camera);

                    // Update reference
                    if (method == 2) {
                        reference.at<float>(i, j) = input.at<float>(i, j);
                    }
                    if (method == 3) {
                        reference.at<float>(i, j) = reference.at<float>(i, j) - event_num * thresholdmap.at<float>(i, j);
                    }

                    // Update threshold map (Increase)
                    thresholdmap.at<float>(i, j) = (1 + adapt_thresh_coef_shift) * thresholdmap.at<float>(i, j);
                }
                else {
                    // Update threshold map (Decrease)
                    thresholdmap.at<float>(i, j) = (1 - adapt_thresh_coef_shift) * thresholdmap.at<float>(i, j);
                }

                // Update reference (Copy Input)
                if (method == 1) {
                    for (int m = 0; m < blocksize; m++) {
                        for (int n = 0; n < blocksize; n++) {
                            reference.at<float>(i + m, j + n) = input.at<float>(i + m, j + n);
                        }
                    }
                }
            }
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");

    float time_gap = 1000000. / 100.;
    SimulationInterface sim(time_gap, 5, 1, 1, 0.2, 0, 3);

    ros::spin();

    return 0;
}
