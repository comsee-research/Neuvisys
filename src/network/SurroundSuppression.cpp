//
// Created by antony on 03/05/2022
//

#include "SurroundSuppression.hpp"
namespace fs = std::filesystem;

/**
 * Creates an object of the class Surround Suppression. 
 * @param networkPath 
 * @param path 
 * @param network 
 */
SurroundSuppression::SurroundSuppression(const std::string &networkPath, const paths_container &path, NetworkHandle& network) : m_network(network) {
    for(const auto& p: path)
    {
        m_path.emplace_back(p);
    }
    m_networkPath = networkPath;

    for(int layer=0; layer<m_network.getNetworkConfig().getLayerConnectivity().size(); layer++){
        std::vector<receptivefield> to_concatenate;
        int numberOfNeuronsInLayer = m_network.getNetworkConfig().getLayerConnectivity()[layer].patches[0].size()
                * m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[0]
                * m_network.getNetworkConfig().getLayerConnectivity()[layer].patches[1].size()
                * m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[1]
                * m_network.getNetworkConfig().getLayerConnectivity()[layer].patches[2].size()
                * m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[2];
        for(int j=0; j<numberOfNeuronsInLayer; j++){
            receptivefield temp;
            getReceptiveField(layer, j, temp);
            to_concatenate.emplace_back(temp);
        }
        m_neuronsReceptiveField.emplace_back(to_concatenate);
    }
    m_time_gap=1000;
    m_log_threshold=0;
    m_map_threshold=0.4;
    m_n_max=5;
    m_adapt_thresh_coef_shift=0.05;
    m_timestamp_noise=50;
}

/**
 * Trains the network on the dataset.
 * @param typeOfTraining 
 * @param numberOfTimes 
 * @param epochs 
 */
void SurroundSuppression::train(const std::string &typeOfTraining, int numberOfTimes, int epochs){
    if(typeOfTraining==m_network.getSimpleNeuronConfig().STDP_LEARNING){
        auto rng = std::default_random_engine {};
        std::cout << "Training is about to start..." << std::endl;
        events_list events;
        for(int j=0; j<epochs; j++){
            std::shuffle(std::begin(m_path), std::end(m_path), rng);
            std::cout << "It's epoch number : " << j << " !" << std::endl;
            for(int i=0; i<m_path.size();i++){
                    std::cout << "Training of event folder number : " << i+1 << " !" << std::endl;
                    while (m_network.loadEvents(events, numberOfTimes)) {
                    m_network.feedEvents(events);
                }
                m_network.save(m_path[i], numberOfTimes);
                events.clear();
                if(i!=m_path.size()-1){
                    m_network.setEventPath(m_path[i+1]);
                }
            }
            m_network.setEventPath(m_path[0]);
        }
    }
    else{

        std::cout << "Please, verify that the type of learning is correct." << std::endl;
    }        
}

/**
 * Identifies the receptive field of a simple cell.
 * @param layer 
 * @param neuron_id 
 * @param to_return 
 */
void SurroundSuppression::getReceptiveField(int layer, int neuron_id, receptivefield &to_return) {
    if(layer>=m_network.getNetworkConfig().getLayerConnectivity().size() || layer < 0){
        std::cout << "Please verify the value you gave to the parameter 'layer'." << std::endl;
    }
    else{
        int totalNumberOfNeurons = m_network.getNetworkConfig().getLayerConnectivity()[layer].patches[0].size()
                * m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[0]
                * m_network.getNetworkConfig().getLayerConnectivity()[layer].patches[1].size()
                * m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[1]
                * m_network.getNetworkConfig().getLayerConnectivity()[layer].patches[2].size()
                * m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[2];
        if(neuron_id<0 || neuron_id>=totalNumberOfNeurons){
            std::cout << "Please verify the value you gave to the parameter 'neuron_id'." << std::endl;
        }
        else{
            int id = neuron_id;
            int actual_x = 0;
            while(id>=0){
                id=id - m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[1] * m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[2];
                actual_x++;
            }
            actual_x = actual_x -1;
            id+=m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[1] * m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[2];
            int actual_y = 0;
            while(id>=0){
                id = id - m_network.getNetworkConfig().getLayerConnectivity()[layer].sizes[2];
                actual_y++;            
            }
            actual_y = actual_y -1;
            int x_minus=0; 
            int y_minus=0;
            int x_plus=0;
            int y_plus=0;
            int neursizes_x = m_network.getNetworkConfig().getLayerConnectivity()[layer].neuronSizes[0][0];
            int overlap_x = m_network.getNetworkConfig().getLayerConnectivity()[layer].neuronOverlap[0];
            if(overlap_x!=0)
            {
                x_minus= (actual_x)*(neursizes_x-overlap_x);
            }
            else
            {
                x_minus=(actual_x)*neursizes_x;
            }
            
            x_plus = x_minus + m_network.getNetworkConfig().getLayerConnectivity()[layer].neuronSizes[0][0];

            int neursizes_y = m_network.getNetworkConfig().getLayerConnectivity()[layer].neuronSizes[0][1];
            int overlap_y = m_network.getNetworkConfig().getLayerConnectivity()[layer].neuronOverlap[1];
            if(overlap_y!=0)
            {
                y_minus= (actual_y) *(neursizes_y-overlap_y);
            }
            else
            {
                y_minus=(actual_y)*neursizes_y;
            }
            
            y_plus = y_minus + m_network.getNetworkConfig().getLayerConnectivity()[layer].neuronSizes[0][1];
            to_return.x_minus = x_minus;
            to_return.x_plus = x_plus;
            to_return.y_minus = y_minus;
            to_return.y_plus = y_plus;
        }
    }
}

/**
 * Generates sequences of an increasing number of bars of the same length.
 * @param ev 
 * @param yPositionStart 
 * @param lengthsOfBars 
 * @param speed 
 * @param num_disparities 
 * @param nbPass 
 * @param pathToImgs 
 * @param thickness 
 * @param angle 
 */
void SurroundSuppression::multiBars1Length(events_sequences &ev, const std::vector<int> &yPositionStart, const std::vector<int> &lengthsOfBars, int speed, int num_disparities, int nbPass, const std::string &pathToImgs, int thickness, int angle) {
    std::vector<int> disparities;
    for(int i=0; i<num_disparities; i++) {
        std::filesystem::create_directory(pathToImgs + std::to_string(i));
        int frame = 0;
        int framerate = 1000;
        int shift = 0;
        int once = 0;
        bool ok = true;
        if(speed >0 ) {
            while(ok) {
                if(once==0 || (once<=i && (shift + disparities.at(once-1))>=10)) {
                    if(once==0) {
                        disparities.push_back(0);
                    }
                    else {
                        disparities.push_back(disparities.at(once-1)-10);
                    }
                    once+=1;
                }

                image_matrix img(m_network.getNetworkConfig().getVfHeight(), m_network.getNetworkConfig().getVfWidth(), CV_8UC3,cv::Scalar(128, 128, 128));
                shift = int(frame * float(float(speed)/framerate));
                for(int j=0; j<disparities.size(); j++) {
                    cv::Point p1(disparities.at(j)+shift, yPositionStart.at(0));
                    cv::Point p2(disparities.at(j)+shift, yPositionStart.at(0)+lengthsOfBars.at(0));
                    cv::line(img, p1, p2, cv::Scalar(255, 255, 255),thickness, cv::LINE_8);
                }
                if(once-1==i) {
                    if(shift + disparities.at(i) > m_network.getNetworkConfig().getVfWidth() + 5 ) {
                        ok=false;
                    }
                }
                cv::Point center((img.cols - 1) / 2.0, (img.rows - 1) / 2.0);
                image_matrix rot = getRotationMatrix2D(center, angle, 1.0); 
                image_matrix newimg;
                cv::warpAffine(img, newimg, rot, cv::Size(img.cols, img.rows), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT,cv::Scalar(0,0,0));
                std::ostringstream path;
                path << pathToImgs << i << "/img_" << frame << ".png";
                cv::imwrite(path.str(), newimg);
                frame+=1;
            }
            disparities.clear();
        }
        std::string new_path = pathToImgs + std::to_string(i) + "/";
        std::cout << new_path << std::endl;
        events_list temp_ev;
        createEvents(new_path, temp_ev, nbPass);
        ev.emplace_back(temp_ev);
    }
}

/**
 * Generates sequences of one bar with an increasing length.
 * @param ev 
 * @param lengthsOfBars 
 * @param yPositionStart 
 * @param angle 
 * @param speed 
 * @param nbPass 
 * @param pathToImgs 
 * @param thickness 
 */
void SurroundSuppression::oneBarMultiLengths(events_sequences &ev, const std::vector<int> &lengthsOfBars, const std::vector<int> &yPositionStart, int angle, int speed, int nbPass, const std::string &pathToImgs, int thickness) {
    for(int i=0; i<lengthsOfBars.size(); i++)
    {
        std::filesystem::create_directory(pathToImgs + std::to_string(i));
        int frame = 0;
        int framerate = 1000;
        int shift = 0;
        if(speed >0 )
        {
            int x = 0;
            while(shift < m_network.getNetworkConfig().getVfWidth() + 5 )
            {
                image_matrix img(m_network.getNetworkConfig().getVfHeight(), m_network.getNetworkConfig().getVfWidth(), CV_8UC3,cv::Scalar(128, 128, 128));
                shift = int(frame * float(float(speed)/framerate));
                cv::Point p1(x+shift, yPositionStart.at(i));
                cv::Point p2(x+shift, yPositionStart.at(i)+lengthsOfBars.at(i));
                cv::line(img, p1, p2, cv::Scalar(255, 255, 255),thickness, cv::LINE_8);
                cv::Point center((img.cols - 1) / 2.0, (img.rows - 1) / 2.0);
                image_matrix rot = getRotationMatrix2D(center, angle, 1.0);
                image_matrix newimg;
                cv::warpAffine(img, newimg, rot, cv::Size(img.cols, img.rows), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT,cv::Scalar(0,0,0));
                std::ostringstream path;
                path << pathToImgs << i << "/img_" << frame << ".png";
                cv::imwrite(path.str(), newimg);
                frame+=1;
            }
        }
        else if (speed < 0)
        {
            int x = m_network.getNetworkConfig().getVfWidth()-1;
            while(shift > -(m_network.getNetworkConfig().getVfWidth()+5))  {
                image_matrix img(m_network.getNetworkConfig().getVfHeight(), m_network.getNetworkConfig().getVfWidth(), CV_8UC3,cv::Scalar(128, 128, 128));
                shift = int(frame * float(float(speed)/framerate));
                cv::Point p1(x+shift, yPositionStart.at(i));
                cv::Point p2(x+shift, yPositionStart.at(i)+lengthsOfBars.at(i));
                cv::line(img, p1, p2, cv::Scalar(255, 255, 255),thickness, cv::LINE_8);
                cv::Point center((img.cols - 1) / 2.0, (img.rows - 1) / 2.0);
                image_matrix rot = getRotationMatrix2D(center, angle, 1.0);      //Mat object for storing after rotation
                image_matrix newimg;
                cv::warpAffine(img, newimg, rot, cv::Size(img.cols, img.rows), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT,cv::Scalar(0,0,0));
                std::ostringstream path;
                path << pathToImgs << i << "/img_" << frame << ".png";
                cv::imwrite(path.str(), newimg);
                frame+=1;
             }
        }
        std::string new_path = pathToImgs + std::to_string(i) + "/";
        std::cout << new_path << std::endl;
        events_list temp_ev;
        createEvents(new_path, temp_ev, nbPass);
        ev.emplace_back(temp_ev);
    }
}

/**
 * Generate sequences of bars.
 * @param ev 
 * @param numberOfTypesOfBars 
 * @param lengthsOfBars 
 * @param yPositionStart 
 * @param angle 
 * @param speed 
 * @param nbPass 
 */
void SurroundSuppression::generateBars(events_sequences &ev,  int numberOfTypesOfBars, const std::vector<int> &lengthsOfBars, const std::vector<int> &yPositionStart, int angle, int speed, int nbPass) {
        if(std::all_of(yPositionStart.cbegin(), yPositionStart.cend(), [](int i){ return i>=0; }) && std::all_of(yPositionStart.cbegin(), yPositionStart.cend(), [this](int i){ return i<m_network.getNetworkConfig().getVfHeight(); })){
            int num_disparities = numberOfTypesOfBars;
            int thickness =1;
            std::string choice;
            std::cout << "Do you want to enter the mode to generate multiple bars of the same length in the same frame?" << std::endl;
            std::cin >> choice;
            std::string path = m_network.getNetworkConfig().getNetworkPath() + "generateSequences/";
            std::filesystem::file_status s = std::filesystem::file_status{};
            if(std::filesystem::status_known(s) ? std::filesystem::exists(s) : std::filesystem::exists(path)) {
                std::filesystem::remove_all(path); 
            }
            std::filesystem::create_directory(path);
            if(choice=="yes" || choice=="y") {
                int fac;
                fac = 34;
                num_disparities = fac;
                multiBars1Length(ev, yPositionStart, lengthsOfBars, speed, num_disparities, nbPass, path, thickness, angle);
            }
            else {
                oneBarMultiLengths(ev, lengthsOfBars, yPositionStart, angle, speed, nbPass, path, thickness);
            }
        }
}

/**
 * Finds the position where to start the bar to keep it centered on the tracked neuron regardless of the length.
 * @param middle 
 * @param value 
 * @param positionStart 
 * @param listOfLengths 
 * @param numberOfBars 
 */
void SurroundSuppression::findCenteredBarsPosition(int middle, int value, std::vector<int> &positionStart, std::vector<int> &listOfLengths, int numberOfBars) {
    int start;
    int offset;
    int n_ = numberOfBars;
    int border;
    for(int i=0; i<n_; i++) { 
        offset = value * (i+1); 
        border = middle-int(offset/2)+ m_network.getNetworkConfig().getLayerConnectivity()[0].patches[1][0];
        if(i>=0 && border>=0) {
            start = border;  
        }      
        else {
            start = 0;
        }
        if(start>=0 && start+offset<m_network.getNetworkConfig().getVfHeight()) {
            listOfLengths.emplace_back(offset);
            positionStart.emplace_back(start);
        }
    }
}

/**
 * Finds the center of the bar according to its orientation.
 * @param middle 
 * @param angle 
 */
void SurroundSuppression::findMiddlePointOrdinate(int &middle, int angle) {
    int real_track_coord = m_network.getSimpleNeuronConfig().POTENTIAL_TRACK[0]*m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[1] *
            m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[2] + m_network.getSimpleNeuronConfig().POTENTIAL_TRACK[1]*m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[2];
    int to_track_x = ((m_neuronsReceptiveField[0][real_track_coord]).x_plus+(m_neuronsReceptiveField[0][real_track_coord]).x_minus)/2 +
            m_network.getNetworkConfig().getLayerConnectivity()[0].patches[0][0];
    int to_track_y = ((m_neuronsReceptiveField[0][real_track_coord]).y_plus+(m_neuronsReceptiveField[0][real_track_coord]).y_minus)/2 +
            m_network.getNetworkConfig().getLayerConnectivity()[0].patches[1][0];
    double sin_rad = sin(angle*M_PI/180);
    double cos_rad = cos(angle*M_PI/180);
    double center_x = m_network.getNetworkConfig().getVfWidth()/2;
    double center_y = m_network.getNetworkConfig().getVfHeight()/2;
    double num = to_track_y + (sin_rad/cos_rad)*to_track_x - (sin_rad/cos_rad)*(1-cos_rad)*center_x + (sin_rad/cos_rad)*(sin_rad*center_y)-(sin_rad*center_x)-(1-cos_rad)*center_y;
    double den = sin_rad*sin_rad/cos_rad + cos_rad;
    middle = int(floor(num/den)) - m_network.getNetworkConfig().getLayerConnectivity()[0].patches[1][0];
}

/**
 * Verifies if the parameters to launch the evaluation of Surround Suppression are safe.
 * @return true 
 * @return false 
 */
bool SurroundSuppression::verifyIfSafe() {
    int numberOfMax_nx = m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[0];
    int numberOfMax_ny = m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[1];
    if(numberOfMax_nx <= m_network.getSimpleNeuronConfig().POTENTIAL_TRACK[0] || numberOfMax_ny <= m_network.getSimpleNeuronConfig().POTENTIAL_TRACK[1]) {
        std::cout << "The coordinates in \"POTENTIAL_TRACK\" are not right. Program aborted." << std::endl;
        return false;
    }
    int n = m_network.getSimpleNeuronConfig().POTENTIAL_TRACK[0] *m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[1]*m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[2] + m_network.getSimpleNeuronConfig().POTENTIAL_TRACK[1] *m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[2];
    int x_minus=(m_neuronsReceptiveField[0][n]).x_minus;
    int x_plus=(m_neuronsReceptiveField[0][n]).x_plus;
    int y_minus=(m_neuronsReceptiveField[0][n]).y_minus;
    int y_plus=(m_neuronsReceptiveField[0][n]).y_plus;
    std::cout << "The neurons with indexes " << n << "-" << n + m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[2] -1 << " (last one included) should have their stats data being saved." << std::endl;
    std::cout << "The receptive field of this neuron is : " << x_minus << " - " << x_plus << " X " << y_minus << " - " << y_plus << std::endl << std::endl;
    if(m_network.getSimpleNeuronConfig().STDP_LEARNING!="excitatory" && m_network.getSimpleNeuronConfig().STDP_LEARNING!="inhibitory" && m_network.getSimpleNeuronConfig().STDP_LEARNING!="all") {
        return true;
    }
    else {
        std::cout << "Change the parameter \"STDP_LEARNING\" for the program to work." << std::endl;
        return false;
    }
}

/**
 * Specifies the simulation choices for the evaluation of Surround Suppression.
 * @param choice 
 * @param simulation 
 * @param n_ 
 */
void SurroundSuppression::simulationChoices(std::string &choice, int &simulation, int &n_) {
    std::string str_n;
    int max_n_ = 150;
    std::cout << "Type the number of bars that will be generated." << std::endl;
    std::cin >> str_n;
    while(std::stoi(str_n)<=0 || std::stoi(str_n)>=max_n_) {
        std::cout << "Please, type a number of bar that is comprised between 1 and " << max_n_ << "." << std::endl;
        std::cin >> str_n;
    }
    n_ = std::stoi(str_n);
    std::cout << "Do you want the bar to be centered, or not?" << std::endl;
    std::cin >> choice;
    std::cout << "How many simulations do you want to run ?" << std::endl;
    std::cin >> simulation;
}

/**
 * Evaluates the responses of the network to different gratings stimuli.
 */
void SurroundSuppression::evaluateResponsesOnStimuli() {
    bool ok = verifyIfSafe();
    if(ok) {
        int value = 1;
        int nbPass = 1;
        int speedValue = 150;
        int n_;
        int middle;
        std::string choice;
        int simulation;
        simulationChoices(choice, simulation, n_);
        if(choice=="yes" || choice=="y") {
            int angle = 0; // {0, 23, 45, 68, 90, 113, 135, 158, 180, 203, 225, 248, 270, 293, 315, 338};
            findMiddlePointOrdinate(middle, angle);
            std::vector<int> listOfLengths;
            std::vector<int>positionStart;
            findCenteredBarsPosition(middle, value, positionStart, listOfLengths, n_);
            for(int sim=0; sim < simulation; sim++) {
                std::cout << "Running simulation " << sim+1 << "/" << simulation << "." << std::endl;
                events_sequences ev;
                generateBars(ev, n_, listOfLengths, positionStart, angle, speedValue, nbPass);
                for(int i=0; i< ev.size(); i++)
                {
                    if(listOfLengths.size() != 1 ) {
                        std::cout << "Evaluating bar number " << i << " that has a length of " << listOfLengths.at(i) << " and goes from ordinate " << positionStart.at(i) << " to " << positionStart.at(i) + listOfLengths.at(i) << std::endl;
                    }
                    else {
                        std::cout << "Evaluating bar number " << i << " that has a length of " << listOfLengths.at(0) << " and goes from ordinate " << positionStart.at(0) << " to " << positionStart.at(0) + listOfLengths.at(0) << std::endl;
                    }
                    m_network.feedEvents(ev.at(i)); 
                    m_network.saveStatistics(sim, i);
                }
            }
        }
        else {
            int index_neur = m_network.getSimpleNeuronConfig().POTENTIAL_TRACK[0] * m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[1] * m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[2] + m_network.getSimpleNeuronConfig().POTENTIAL_TRACK[1] * m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[2];
            int depth = m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[2];
            int num_angles = 8;
            std::vector<std::vector<int>> vec(depth, std::vector<int> (num_angles, 0));
            int angles[8] = {0, 23, 45, 68, 90, 113, 135, 158};
            for(int val=0; val<8; val++) {
                int angle = angles[val];
                std::cout << "Angle = " << angle << std::endl;
                findMiddlePointOrdinate(middle, angle);
                std::vector<int> listOfLengths;
                std::vector<int> positionStart;
                findCenteredBarsPosition(middle, value, positionStart, listOfLengths, n_);
                for(int sim=0; sim < simulation; sim++) {
                    std::cout << "Running simulation " << sim+1 << "/" << simulation << "." << std::endl;
                    events_sequences ev;
                    generateBars(ev,n_,listOfLengths,positionStart,angle, speedValue, nbPass);
                    for(int i=0; i< ev.size(); i++) {
                        std::cout << "Evaluating bar number " << i << " that has a length of " << listOfLengths.at(i) << " and goes from ordinate " << positionStart.at(i) << " to " << positionStart.at(i) + listOfLengths.at(i) << std::endl;
                        m_network.feedEvents(ev.at(i)); 
                        m_network.saveStatistics(sim, i);
                    }
                }
                for(int neur = 0; neur < m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[2]; neur++) {
                    vec.at(neur).at(val)=m_network.getNeuron(index_neur+neur,0).get().getTrackingSpikeTrain().size();
                }
            }
            for(int neur = 0; neur < m_network.getNetworkConfig().getLayerConnectivity()[0].sizes[2]; neur++) {
                int angle_index = std::max_element(vec.at(neur).begin(),vec.at(neur).end()) - vec.at(neur).begin();
                m_network.assignOrientation(neur,angles[angle_index]);
            }
            m_network.saveStatistics(simulation, 0, true);
        }
    } 
}

/**
 * Calculates the logarithmic value of the frame to simulate events.(fro the ESIM simulator)
 * @param frame 
 * @param new_frame 
 */
void SurroundSuppression::convertFrame(image_matrix frame, image_matrix& new_frame) {
    image_matrix channels[3];
    cv::split(frame,channels);    
    for(int i=0; i<m_network.getNetworkConfig().getVfWidth(); i++) {
        for(int j=0; j<m_network.getNetworkConfig().getVfHeight(); j++) {
            new_frame.at<double>(cv::Point(i, j)) = (int)channels[0].at<uchar>(cv::Point(i, j))*0.299 + (int)channels[1].at<uchar>(cv::Point(i, j))*0.587 + (int)channels[2].at<uchar>(cv::Point(i, j))*0.114;
            if(new_frame.at<double>(cv::Point(i, j)) > m_log_threshold) {
                new_frame.at<double>(cv::Point(i, j)) = log(new_frame.at<double>(cv::Point(i, j)));
            }
        }
    }
}
/**
 * Generates events.
 * @param events 
 * @param delta_b 
 * @param thresh 
 * @param frame_id 
 * @param x 
 * @param y 
 * @param polarity 
 */
void SurroundSuppression::writeEvents(events_list &events, float delta_b, float thresh, float frame_id, int x, int y, int polarity) {
    std::random_device rd;
    std::default_random_engine generator(rd());

    std::uniform_int_distribution<int> distribution(floor(-m_timestamp_noise/2),floor(m_timestamp_noise/2)+1);
    int nb_event;
    int moddiff = int(delta_b/thresh);
    if(moddiff > m_n_max || moddiff <0) {
        nb_event = m_n_max;
    }
    else {
        nb_event = moddiff;
    }
    for(int e=0; e<nb_event; e++) {
        int random_val = distribution(generator);
        int timestamp = ( ( (m_time_gap * (e+1) * thresh) /delta_b) + m_time_gap * frame_id + random_val );
        if(timestamp < 0) {
            timestamp = 0;
        }
        events.emplace_back(Event(timestamp,x,y,polarity,false));
        Event ev(timestamp,x,y,polarity,false);
    }
}

/**
 * Converts successive frames to events if a brightness threshold is locally reached.
 * @param frame_id 
 * @param frame 
 * @param reference 
 * @param threshold_map 
 * @param events 
 */
void SurroundSuppression::frameToEvents(int frame_id, image_matrix frame, image_matrix reference, image_matrix &threshold_map, events_list &events) {
    image_matrix delta = frame - reference;
    for(int i=0; i<m_network.getNetworkConfig().getVfWidth(); i++) {
        for(int j=0; j<m_network.getNetworkConfig().getVfHeight(); j++) {
            if(delta.at<double>(cv::Point(i,j)) > threshold_map.at<double>(cv::Point(i,j))) {
                writeEvents(events, delta.at<double>(cv::Point(i,j)), threshold_map.at<double>(cv::Point(i,j)), frame_id, i, j, 1);
                threshold_map.at<double>(cv::Point(i,j)) *= (1+m_adapt_thresh_coef_shift);
            }
            else if(delta.at<double>(cv::Point(i,j)) < -threshold_map.at<double>(cv::Point(i,j))) {
                writeEvents(events, -delta.at<double>(cv::Point(i,j)), threshold_map.at<double>(cv::Point(i,j)), frame_id, i, j, 0);
                threshold_map.at<double>(cv::Point(i,j)) *= (1+m_adapt_thresh_coef_shift);
            }
            else if(delta.at<double>(cv::Point(i,j)) <= threshold_map.at<double>(cv::Point(i,j)) || delta.at<double>(cv::Point(i,j)) >= -threshold_map.at<double>(cv::Point(i,j))) {
                threshold_map.at<double>(cv::Point(i,j)) *= (1-m_adapt_thresh_coef_shift);
            }
        }
    }
}

/**
 * Creates events sequence from a folder of frames of a moving object.
 * @param pathToFrames 
 * @param events 
 * @param nbPass 
 */
void SurroundSuppression::createEvents(const std::string &pathToFrames, events_list &events, int nbPass) {
    image_matrix threshold_map(cv::Size(m_network.getNetworkConfig().getVfWidth(), m_network.getNetworkConfig().getVfHeight()), CV_64FC1, cv::Scalar(m_map_threshold));
    paths_container frames;
    for (const auto & frame : std::filesystem::directory_iterator{pathToFrames}) {
        frames.emplace_back(frame.path().filename().stem().string());
    }
    std::sort(frames.begin(), frames.end(), compareImgName);
    std::string extension = ".png";
    image_matrix init = cv::imread(pathToFrames + frames[0] + extension);
    image_matrix reference(cv::Size(m_network.getNetworkConfig().getVfWidth(), m_network.getNetworkConfig().getVfHeight()), CV_64FC1);
    convertFrame(init,reference);
    int sze = frames.size();
    int i =1;
    while(i<frames.size()) {
        std::string newpath = pathToFrames + frames[i] + extension;
        init = cv::imread(pathToFrames + frames[i] + extension);
        if (!init.data) {
            std::cout << "Could not open or find" << " the image" << std::endl;
        }
        image_matrix frame(cv::Size(m_network.getNetworkConfig().getVfWidth(), m_network.getNetworkConfig().getVfHeight()), CV_64FC1, cv::Scalar(0));
        convertFrame(init,frame);
        
        frameToEvents(i,frame,reference,threshold_map,events);
        for(int v=0; v<m_network.getNetworkConfig().getVfWidth(); v++) {
            for(int w = 0; w<m_network.getNetworkConfig().getVfHeight(); w++) {
                reference.at<double>(cv::Point(v,w)) = frame.at<double>(cv::Point(v,w));
            }
        }
        i++;
    }
    std::sort(events.begin(),events.end());
    int size = events.size();
    long firstTimestamp = static_cast<long>((events.at(0)).timestamp());
    long actualLast = static_cast<long>((events.at(size-1)).timestamp());
    long value = actualLast - firstTimestamp;
    
    for(i=0; i<nbPass; i++)
    {
        for (int j=0; j<size; j++)
        {
            if(i>0)
            {
                Event ev( (events.at(j)).timestamp() + static_cast<long>(i) * (value), (events.at(j)).x(), (events.at(j)).y(), (events.at(j)).polarity(), (events.at(j)).camera());
                events.push_back(ev);
            }
            else
            {
                break;
            }
        }
    }
}

/**
 * Re-initializes randomly the lateral inhibition weights.
 */
void SurroundSuppression::randomLateralInit() {
    m_network.lateralRandom();
}

/**
 * Shuffles one or all inhibition weights.
 * @param c 
 */
void SurroundSuppression::shuffleInhibition(int c) {
    m_network.inhibitionShuffle(c);
}

/**
 * Orders correctly the png files generated by the class.
 * @param file_1 
 * @param file_2 
 * @return true 
 * @return false 
 */
bool SurroundSuppression::compareImgName(std::string file_1, std::string file_2) {
    if(file_1.at(0)=='i' && file_1.at(1)=='m' && file_1.at(2) == 'g' && file_1.at(3)== '_' && file_2.at(0)=='i' && file_2.at(1)=='m' && file_2.at(2) == 'g' && file_2.at(3)== '_') {
        file_1.erase(0,4);
        file_2.erase(0,4);
        return(std::stoi(file_1) < std::stoi(file_2));
    }
    else {
        return false;
    }
}