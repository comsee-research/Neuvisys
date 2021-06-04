//
// Created by thomas on 04/06/2021.
//

#include "FrameToEvents.hpp"

FrameToEvents::FrameToEvents(int n_max, int blocksize, int log_threshold, float map_threshold, float adapt_thresh_coef_shift, int method) : n_max(n_max), blocksize(blocksize), map_threshold(map_threshold), log_threshold(log_threshold), adapt_thresh_coef_shift(adapt_thresh_coef_shift), method(method) {

}

void FrameToEvents::frameConversion(int count, const std::string &topic, const ros::MessageEvent<sensor_msgs::Image const> &frame, cv::Mat &reference, cv::Mat &input, cv::Mat &thresholdmap, cv::Mat &eim, std::vector<Event> &events, int camera) {
    try {
        cv::cvtColor(cv_bridge::toCvCopy(frame.getMessage(), sensor_msgs::image_encodings::BGR8)->image, input, CV_BGR2GRAY);
        input.convertTo(input, CV_32F);
        logFrame(input);
        if (count == 0) {
            thresholdmap = cv::Mat(input.size(), CV_32F);
            thresholdmap = map_threshold;
            reference = input.clone();
        } else {
            convertFrameToEvent(input, reference, thresholdmap, events, frame.getMessage()->header.stamp.toSec(), camera);

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
            prevTime = frame.getMessage()->header.stamp.toSec();
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void FrameToEvents::convertFrameToEvent(const cv::Mat& input, cv::Mat &reference, cv::Mat &thresholdmap, std::vector<Event> &events, long time, int camera) const {
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
                int event_num = write_event(events, delta_B, thresholdmap.at<float>(i + i_shift, j + j_shift), time, i + i_shift, j + j_shift, 1, camera);

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
                int event_num = write_event(events, delta_B, thresholdmap.at<float>(i + i_shift, j + j_shift), time, i + i_shift, j + j_shift, 0, camera);

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

void FrameToEvents::logFrame(cv::Mat &frame) const {
    for (int i = 0; i < frame.rows; i++) {
        for (int j = 0; j < frame.cols; j++) {
            if (frame.at<float>(i, j) > log_threshold) {
                frame.at<float>(i, j) = log(frame.at<float>(i, j));
            }
        }
    }
}

int FrameToEvents::write_event(std::vector<Event> &events, float delta_B, float threshold, long time, int x, int y, int polarity, int camera) const {
    int moddiff = delta_B / threshold;
    int evenum;

    if (moddiff > n_max) {
        evenum = n_max;
    } else {
        evenum = moddiff;
    }

    auto dt = (time - prevTime) * 1e6;

    for (int e = 0; e < evenum; e++) {
        double timestamp = ((dt * (e + 1) * threshold) / delta_B) + time * 1e6;
        events.emplace_back(timestamp, x, y, polarity, camera);
    }

    return evenum;
}

cv::Mat FrameToEvents::eventImage(const cv::Size& size, const std::vector<Event>& events) {
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
