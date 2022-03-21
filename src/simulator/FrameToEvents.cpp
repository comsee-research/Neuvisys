//
// Created by thomas on 04/06/2021.
//

#include "FrameToEvents.hpp"

FrameToEvents::FrameToEvents(int n_max, int blocksize, int log_threshold, float map_threshold,
                             float adapt_thresh_coef_shift, int method, bool saveFrames, bool saveEvents) : n_max(n_max), blocksize(blocksize),
                                                                                                            map_threshold(map_threshold),
                                                                                                            log_threshold(log_threshold),
                                                                                                            adapt_thresh_coef_shift(
                                                                                                                    adapt_thresh_coef_shift),
                                                                                                            method(method),
                                                                                                            m_saveFrames(saveFrames),
                                                                                                            m_saveEvents(saveEvents) {
    m_events = std::vector<Event>();
}

void FrameToEvents::frameConversion(const std::string &topic, const ros::MessageEvent<sensor_msgs::Image const> &frame,
                                    cv::Mat &reference, cv::Mat &thresholdmap, cv::Mat &eim, std::vector<Event> &events, int camera,
                                    bool firstImage) {
    cv::Mat input;
    try {
        cv::cvtColor(cv_bridge::toCvCopy(frame.getMessage(), sensor_msgs::image_encodings::BGR8)->image, input,
                     CV_BGR2GRAY);
        input.convertTo(input, CV_32F);
        if (m_saveFrames) {
            cv::imwrite("/home/thomas/Desktop/frames/" + std::to_string(m_iterations) + ".png", input);
        }

        logFrame(input);
        auto time = static_cast<long>(frame.getMessage()->header.stamp.toNSec() / 1000);
        if (firstImage) {
            thresholdmap = cv::Mat(input.size(), CV_32F);
            thresholdmap = map_threshold;
            reference = input.clone();
        } else {
            convertFrameToEvent(input, reference, thresholdmap, events, time, camera);

            if (!events.empty()) {
                if (m_saveEvents) {
                    m_events.insert(m_events.end(), events.begin(), events.end());
                }

                eim = eventImage(input.size(), events);
                cv::imshow(topic, eim);
            }
        }
        prevTime = time;
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    std::sort(events.begin(), events.end());
    ++m_iterations;
}

void FrameToEvents::convertFrameToEvent(const cv::Mat &inputFrame, cv::Mat &reference, cv::Mat &thresholdmap,
                                        std::vector<Event> &events, long
                                        time, int camera) const {
    for (int row = 0; row < inputFrame.rows; row = row + blocksize) {
        for (int col = 0; col < inputFrame.cols; col = col + blocksize) {
            // Find local maxima --
            // Set blocksize to 0 to disable Local Inhibition
            float diff;
            float diff_ = 0;
            int row_shift = 0;
            int col_shift = 0;
            for (int row_block = 0; row_block < blocksize; row_block++) {
                for (int col_block = 0; col_block < blocksize; col_block++) {
                    diff = abs(inputFrame.at<float>(row + row_block, col + col_block) -
                               reference.at<float>(row + row_block, col + col_block));
                    if (diff > diff_) {
                        diff_ = diff;
                        row_shift = row_block;
                        col_shift = col_block;
                    }
                }
            }

            // delta_B: Brightness Difference
            float delta_B = inputFrame.at<float>(row + row_shift, col + col_shift) -
                            reference.at<float>(row + row_shift, col + col_shift);

            if (delta_B > thresholdmap.at<float>(row + row_shift, col + col_shift)) {
                int event_num = write_event(events, delta_B, thresholdmap.at<float>(row + row_shift, col + col_shift),
                                            time, col + col_shift, inputFrame.rows - row + row_shift, true, camera);

                // Update reference
                if (method == 2) {
                    reference.at<float>(row, col) = inputFrame.at<float>(row, col);
                }
                if (method == 3) {
                    reference.at<float>(row, col) =
                            reference.at<float>(row, col) + static_cast<float>(event_num) * thresholdmap.at<float>(row, col);
                }

                // Update threshold map (Increase)
                thresholdmap.at<float>(row, col) *= (1 + adapt_thresh_coef_shift);
            } else if (delta_B < -thresholdmap.at<float>(row + row_shift, col + col_shift)) {
                int event_num = write_event(events, -delta_B, thresholdmap.at<float>(row + row_shift, col + col_shift),
                                            time, col + col_shift, inputFrame.rows - row + row_shift, false, camera);

                // Update reference
                if (method == 2) {
                    reference.at<float>(row, col) = inputFrame.at<float>(row, col);
                }
                if (method == 3) {
                    reference.at<float>(row, col) =
                            reference.at<float>(row, col) - static_cast<float>(event_num) * thresholdmap.at<float>(row, col);
                }

                // Update threshold map (Increase)
                thresholdmap.at<float>(row, col) *= (1 + adapt_thresh_coef_shift);
            } else {
                // Update threshold map (Decrease)
                thresholdmap.at<float>(row, col) *= (1 - adapt_thresh_coef_shift);
            }

            // Update reference (Copy Input)
            if (method == 1) {
                for (int m = 0; m < blocksize; m++) {
                    for (int n = 0; n < blocksize; n++) {
                        reference.at<float>(row + m, col + n) = inputFrame.at<float>(row + m, col + n);
                    }
                }
            }
        }
    }
}

void FrameToEvents::logFrame(cv::Mat &frame) const {
    for (int i = 0; i < frame.rows; i++) {
        for (int j = 0; j < frame.cols; j++) {
            if (frame.at<float>(i, j) > static_cast<float>(log_threshold)) {
                frame.at<float>(i, j) = log(frame.at<float>(i, j));
            }
        }
    }
}

int FrameToEvents::write_event(std::vector<Event> &events, float delta_B, float threshold, long time, int x, int y,
                               bool polarity, int camera) const {
    int moddiff = static_cast<int>(delta_B / threshold);
    int evenum;

    if (moddiff > n_max) {
        evenum = n_max;
    } else {
        evenum = moddiff;
    }

    auto dt = time - prevTime;

    for (int e = 0; e < evenum; e++) {
        long timestamp = static_cast<long>(((dt * (e + 1) * threshold) / delta_B) + time);
        if (x < 346 && y < 260) {
            events.emplace_back(timestamp, x, y, polarity, camera);
        }
    }

    return evenum;
}

cv::Mat FrameToEvents::eventImage(const cv::Size &size, const std::vector<Event> &events) {
    cv::Mat eim = cv::Mat::zeros(size, CV_8UC3);
    cv::Vec3b color; // B,G,R
    for (Event event: events) {
        if (event.polarity() == 0) {
            color = cv::Vec3b(0, 0, 255);
        } else {
            color = cv::Vec3b(0, 255, 0);
        }
        eim.at<cv::Vec3b>(event.y(), event.x()) = color;
    }
    return eim;
}

void FrameToEvents::saveEventsAsFile(std::string filePath) {
    Util::saveEventFile(m_events, filePath);
}
