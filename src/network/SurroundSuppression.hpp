//
// Created by antony on 03/05/2022
//

/*This is a module to evaluate the phenomenon of surround suppression 
    on multiple neurons with bars of different orientations and speed*/

#ifndef NEUVISYS_DV_SURROUND_SUPPRESSION_HPP 0
#define NEUVISYS_DV_SURROUND_SUPPRESSION_HPP 1

//#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
/*#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>*/
#include <sstream>
#include <algorithm>
#include <random>
#include <cmath>
#include <filesystem>
#include "NetworkHandle.hpp"
#include "Config.hpp"
#define width_ 346
#define height_ 260

struct receptiveFieldDimensions {
    int x_minus;
    int x_plus;
    int y_minus;
    int y_plus;
};

class SurroundSuppression {
    std::vector<std::string> m_path;
    std::string m_networkPath;
    NetworkHandle& m_network;
    NetworkConfig m_networkConf;
    NeuronConfig m_simpleNeuronConf;
    NeuronConfig m_complexNeuronConf;
    NeuronConfig m_criticNeuronConf;
    NeuronConfig m_actorNeuronConf;
    std::vector<std::vector<receptiveFieldDimensions>> m_neuronsReceptiveField; 

public: 
//    SurroundSuppression();
    SurroundSuppression(const std::string &networkPath, std::vector<std::string> path, NetworkHandle& network);
    void Training(const std::string &typeOfTraining, int numberOfTimes, int epochs);
    void getReceptiveField(int layer, int neuron_id, receptiveFieldDimensions &to_return);
    void generateVerticalBars(std::vector<std::vector<Event>> &ev, int id_neur, int numberOfTypesOfBars, std::vector<int> lengthsOfBars, std::vector<int> yPositionStart, int speed, int nbPass);
    void launchTrainingNeurons(int oneOrAll);

private:
    int m_time_gap;
    int m_log_threshold;
    float m_map_threshold;
    int m_n_max;
    float m_adapt_thresh_coef_shift;
    int m_timestamp_noise;
    void convertFrame(cv::Mat frame, cv::Mat& new_frame);
    void writeEvents(std::vector<Event> &events, float delta_b, float thresh, float frame_id, int x, int y, int polarity);
    void frameToEvents(int frame_id, cv::Mat frame, cv::Mat reference, cv::Mat &threshold_map, std::vector<Event> &events);
    void createEvents(const std::string &pathToFrames, std::vector<Event> &events, int nbPass);
    static bool compareImgName(std::string file_1, std::string file_2){
    if(file_1.at(0)=='i' && file_1.at(1)=='m' && file_1.at(2) == 'g' && file_1.at(3)== '_' && file_2.at(0)=='i' && file_2.at(1)=='m' && file_2.at(2) == 'g' && file_2.at(3)== '_')
    {
        file_1.erase(0,4);
        file_2.erase(0,4);
    //    std::cout << "number1 = " << file_1 << std::endl;
        return(stoi(file_1) < stoi(file_2));
    }
    else{
        return false;
    }
    };
};

#endif