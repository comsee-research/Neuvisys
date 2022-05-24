//
// Created by antony on 03/05/2022
//

#include "SurroundSuppression.hpp"
namespace fs = std::filesystem;

SurroundSuppression::SurroundSuppression(const std::string &networkPath, std::vector<std::string> path, NetworkHandle& network) : m_network(network)
{
    for(int i=0; i<path.size(); i++)
    {
    //    std::cout << path[i] << std::endl;
        m_path.emplace_back(path[i]);
    }
    m_networkPath = networkPath;

    m_networkConf = NetworkConfig(networkPath + "configs/network_config.json");
    m_simpleNeuronConf= NeuronConfig(m_networkConf.getNetworkPath() + "configs/simple_cell_config.json", 0);
    m_complexNeuronConf= NeuronConfig(m_networkConf.getNetworkPath() + "configs/complex_cell_config.json", 1);
    m_criticNeuronConf= NeuronConfig(m_networkConf.getNetworkPath() + "configs/critic_cell_config.json", 2);
    m_actorNeuronConf= NeuronConfig(m_networkConf.getNetworkPath() + "configs/actor_cell_config.json", 3);

    for(int layer=0; layer<m_networkConf.getLayerCellTypes().size(); layer++){
        std::vector<struct receptiveFieldDimensions> to_concatenate;
        int numberOfNeuronsInLayer = m_networkConf.getLayerPatches()[layer][0].size() * m_networkConf.getLayerSizes()[layer][0] * m_networkConf.getLayerPatches()[layer][1].size() * m_networkConf.getLayerSizes()[layer][1] * m_networkConf.getLayerPatches()[layer][2].size() * m_networkConf.getLayerSizes()[layer][2];
        for(int j=0; j<numberOfNeuronsInLayer; j++){
            receptiveFieldDimensions temp;
            getReceptiveField(layer, j, temp);
            to_concatenate.emplace_back(temp);
        }
        m_neuronsReceptiveField.emplace_back(to_concatenate);
    }
    m_time_gap=1000;//1e6/1000;
    m_log_threshold=0;
    m_map_threshold=0.4;
    m_n_max=5;
    m_adapt_thresh_coef_shift=0.05;
    m_timestamp_noise=50;
}

void SurroundSuppression::Training(const std::string &typeOfTraining, int numberOfTimes, int epochs){
    if(typeOfTraining==m_simpleNeuronConf.STDP_LEARNING){

        std::cout << "Training is about to start..." << std::endl;
        std::vector<Event> events;
        auto rng = std::default_random_engine {};
        for(int j=0; j<epochs; j++){
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
            std::shuffle(std::begin(m_path), std::end(m_path), rng);
            m_network.setEventPath(m_path[0]);
        }
    }
    else{

        std::cout << "Please, verify that the type of learning is correct." << std::endl;
    }
        
}

void SurroundSuppression::getReceptiveField(int layer, int neuron_id, receptiveFieldDimensions &to_return){
    if(layer>=m_networkConf.getLayerCellTypes().size() || layer <0){
        std::cout << "Please verify the value you gave to the parameter 'layer'." << std::endl;
    }
    else{
        int totalNumberOfNeurons = m_networkConf.getLayerPatches()[layer][0].size() * m_networkConf.getLayerSizes()[layer][0] * m_networkConf.getLayerPatches()[layer][1].size() * m_networkConf.getLayerSizes()[layer][1] * m_networkConf.getLayerPatches()[layer][2].size() * m_networkConf.getLayerSizes()[layer][2];
        if(neuron_id<0 || neuron_id>=totalNumberOfNeurons){
            std::cout << "Please verify the value you gave to the parameter 'neuron_id'." << std::endl;
        }
        else{
            int id = neuron_id;
            int actual_x = 0;
            while(id>=0){
                id=id - m_networkConf.getLayerSizes()[layer][1] * m_networkConf.getLayerSizes()[layer][2];
                actual_x++;
            }
            actual_x = actual_x -1;
            id+=m_networkConf.getLayerSizes()[layer][1] * m_networkConf.getLayerSizes()[layer][2];
            int actual_y = 0;
            while(id>=0){
                id = id - m_networkConf.getLayerSizes()[layer][2];
                actual_y++;            
            }
            actual_y = actual_y -1;
            int x_minus=0; 
            int y_minus=0;
            int x_plus=0;
            int y_plus=0;
            int neursizes_x = m_networkConf.getNeuronSizes()[layer][0];
            int overlap_x = m_networkConf.getNeuronOverlap()[layer][0];
            if(overlap_x!=0)
            {
                x_minus= (actual_x)*(neursizes_x-overlap_x);
            }
            else
            {
                x_minus=(actual_x)*neursizes_x;
            }
            
            x_plus = x_minus + m_networkConf.getNeuronSizes()[layer][0];

            int neursizes_y = m_networkConf.getNeuronSizes()[layer][1];
            int overlap_y = m_networkConf.getNeuronOverlap()[layer][1];
            if(overlap_y!=0)
            {
                y_minus= (actual_y) *(neursizes_y-overlap_y);
            }
            else
            {
                y_minus=(actual_y)*neursizes_y;
            }
            
            y_plus = y_minus + m_networkConf.getNeuronSizes()[layer][1];
            to_return.x_minus = x_minus;
            to_return.x_plus = x_plus;
            to_return.y_minus = y_minus;
            to_return.y_plus = y_plus;
        }
    }
}

void SurroundSuppression::generateVerticalBars(std::vector<std::vector<Event>> &ev, int id_neur, int numberOfTypesOfBars, std::vector<int> lengthsOfBars, std::vector<int> yPositionStart, int speed, int nbPass){
    if(numberOfTypesOfBars==lengthsOfBars.size()){
        if(std::all_of(yPositionStart.cbegin(), yPositionStart.cend(), [](int i){ return i>=0; }) && std::all_of(yPositionStart.cbegin(), yPositionStart.cend(), [](int i){ return i<height_; })){
            for(int i=0; i<numberOfTypesOfBars; i++)
            {
                //creation of images for each bar. always make sure that the path exists
                int frame = 0;
                int framerate = 1000;
                int shift = 0;
                if(speed >0 )
                {
                    int x = 0;
                    while(shift < width_ + 5 )
                    {
                        cv::Mat img(height_, width_, CV_8UC3,cv::Scalar(0, 0, 0));
                        shift = int(frame * float(float(speed)/framerate));
                        cv::Point p1(x+shift, yPositionStart.at(i));
                        cv::Point p2(x+shift, yPositionStart.at(i)+lengthsOfBars.at(i));
                        int thickness =4;
                    //    std::cout << "x+ shift = " << x+shift << " ; speed = " << speed << " ; framerate = " << framerate << " ; float of speed / framerate = " << float(float(speed)/framerate) << " ; frame = " << frame << " ; yPositionStart = " << yPositionStart.at(i)+lengthsOfBars.at(i) << " ; length of bars = " << lengthsOfBars.at(i) << std::endl;
                        cv::line(img, p1, p2, cv::Scalar(255, 255, 255),thickness, 8);
                    /*    cv::imshow("Output", img);
                        cv::waitKey(0);*/
                        
                        std::ostringstream path;
                        path << "/home/comsee/Internship_Antony/neuvisys/Events/frames/neur" << id_neur << "/" << i << "/img_" << frame << ".png";
                        cv::imwrite(path.str(), img);
                        frame+=1;
                    }
                }
                else if (speed < 0)
                {
                    int x = width_-1;
                    while(shift > -(width_+5))
                    {
                        cv::Mat img(height_, width_, CV_8UC3,cv::Scalar(0, 0, 0));
                        shift = frame * float(float(speed) / framerate);
                        cv::Point p1(x+shift, yPositionStart.at(i));
                        cv::Point p2(x+shift, yPositionStart.at(i)+lengthsOfBars.at(i));
                        int thickness = 4;
                        cv::line(img, p1, p2, cv::Scalar(255, 255,255),thickness, 8);
                        std::ostringstream path;
                        path << "/home/comsee/Internship_Antony/neuvisys/Events/frames/neur" << id_neur << "/" << i << "/img_" << frame << ".png";
                        cv::imwrite(path.str(), img);
                        frame+=1;
                    }
                }
                //converting saved frames to events
                std::string new_path = "/home/comsee/Internship_Antony/neuvisys/Events/frames/neur" + std::to_string(id_neur) + "/" + std::to_string(i) + "/";
                std::cout << new_path << std::endl;
                std::vector<Event> temp_ev;
                createEvents(new_path, temp_ev, nbPass);
                ev.emplace_back(temp_ev);
            }
            
        }
        else{
            std::cout << "The start position of the ordinate must be between 0 and " << width_ << " excluded." << std::endl;
        }
    }
    else{
        std::cout << "Number of types of bars is wrong." << std::endl;
    }

}

void SurroundSuppression::launchTrainingNeurons()
{
    int numberOfMax_nx = m_networkConf.getLayerSizes()[0][0];
    int numberOfMax_ny = m_networkConf.getLayerSizes()[0][1]; 
    int n_x;
    int n_y;
    int nbPass = 1;
    std::cout << "Type the number of the abscissa x of the neuron(s) you want to evaluate." << std::endl;
    std::cin >> n_x;
    std::cout << "Now, type the number of the ordinate y of the neuron(s) you want to evaluate." << std::endl;
    std::cin >> n_y;
    while(n_x<0 || n_y <0 || n_x>=numberOfMax_nx || n_y >= numberOfMax_ny)
    {
        std::cout << "Please, type an abscissa x that is comprised between 0 and " << numberOfMax_nx << " excluded." << std::endl;
        std::cin >> n_x;
        std::cout << "Now, type an ordinate y that is comprised between 0 and " << numberOfMax_ny << " excluded." << std::endl;
        std::cin >> n_y;
    }
    if(n_x != m_simpleNeuronConf.POTENTIAL_TRACK[0] || n_y != m_simpleNeuronConf.POTENTIAL_TRACK[1])
    {
        std::cout << "WARNING! You are not saving the potentials and weights statistics of the neuron you want to evaluate. Do you want to restart the program? (y/n)" << std::endl;
        char response;
        std::cin >> response;
        while(response!='y' || response!='n')
        {
            std::cout << "Type either 'y' for yes or 'n' for no." << std::endl;
            std::cin >> response;
        }
        if(response=='y')
        {
            std::cout << "Program aborted." << std::endl;
            exit(-1);
        }
        else
        {
            std::cout << "The program will continue then." << std::endl;
        }
    }
    int n = n_x*m_networkConf.getLayerSizes()[0][1]*m_networkConf.getLayerSizes()[0][2] + n_y*m_networkConf.getLayerSizes()[0][2];
    int x_minus=(m_neuronsReceptiveField[0][n]).x_minus;
    int x_plus=(m_neuronsReceptiveField[0][n]).x_plus;
    int y_minus=(m_neuronsReceptiveField[0][n]).y_minus;
    int y_plus=(m_neuronsReceptiveField[0][n]).y_plus;
    std::cout << "The neurons " << n << "-" << n + m_networkConf.getLayerSizes()[0][2] -1 << " (last one included) should have their stats data being saved." << std::endl;
    std::cout << "The receptive field of this neuron is : " << x_minus << " - " << x_plus << " X " << y_minus << " - " << y_plus << std::endl << std::endl;
    std::cout << "The bars are only vertical, thus we will proceed to evaluate all the neurons sharing the same ordinate as receptive field. (regardless of their X or Z value)" << std::endl;
    if(m_simpleNeuronConf.STDP_LEARNING!="excitatory" && m_simpleNeuronConf.STDP_LEARNING!="inhibitory" && m_simpleNeuronConf.STDP_LEARNING!="all")
    {
        int value = 1;
        if(m_networkConf.getNeuronSizes()[0][1]>=value)
        {
            int n_;
            int max_n_ = 150;
            std::cout << "Type the number of bars that will be generated." << std::endl;
            std::cin >> n_;
            while(n_<=0 || n_>=max_n_)
            {
                std::cout << "Please, type a number of bar that is comprised between 0 and " << max_n_ << "." << std::endl;
                std::cin >> n_;
            }
            std::vector<int> listOfLengths;
            int middle = (m_neuronsReceptiveField[0][n].y_plus + m_neuronsReceptiveField[0][n].y_minus)/2;
            int offset;
            int new_offset;
            int start;
            std::vector<int>positionStart;
            for(int i=0; i<n_; i++)
            { 
                offset = value * (i+1);
                start = middle - int(offset/2);                      
                if(start>=0 && start+offset<height_)
                {
                    listOfLengths.emplace_back(offset);
                    positionStart.emplace_back(start);
                }
                else
                {
                    while(start<0 || start+offset>=height_)
                    {
                        offset = offset-1;
                        start = middle - int(offset/2); 
                    }
                    if(m_neuronsReceptiveField[0][n].y_minus==0)
                    {
                        offset = value * (i+1);
                    }                        
                    if(listOfLengths.size()==0)
                    {
                        listOfLengths.emplace_back(offset);
                        if(m_neuronsReceptiveField[0][n].y_minus!=0)
                        {
                            n_ = i+1; // ( = 1)
                        }
                        positionStart.emplace_back(start);
                    }
                    else if(offset!=listOfLengths.at(listOfLengths.size()-1))
                    {
                        listOfLengths.emplace_back(offset);
                        if(m_neuronsReceptiveField[0][n].y_minus!=0)
                        {
                            n_ = i+1; // ( = 1)
                        }
                        positionStart.emplace_back(start);
                    }
                    if(m_neuronsReceptiveField[0][n].y_minus!=0)
                    {
                        break;
                    }
                    
                }
            }
            std::vector<std::vector<Event>> ev;
            int speedValue = 150;
            generateVerticalBars(ev,n,n_,listOfLengths,positionStart,speedValue, nbPass);
            for(int i=0; i< ev.size(); i++)
            {
                std::cout << "Evaluating bar number " << i << " that has a length of " << listOfLengths.at(i) << " and goes from ordinate " << positionStart.at(i) << " to " << positionStart.at(i) + listOfLengths.at(i) << std::endl;
                m_network.feedEvents(ev.at(i)); 
            }
            m_network.save("g",0);
        }
    }
    else
    {
        std::cout << "Change the parameter \"STDP_LEARNING\" for the program to work." << std::endl;
    }
}

void SurroundSuppression::convertFrame(cv::Mat frame, cv::Mat& new_frame)
{
    cv::Mat channels[3];
    cv::split(frame,channels);
    for(int i=0; i<width_; i++)
    {
        for(int j=0; j<height_; j++)
        {
            new_frame.at<double>(cv::Point(i, j)) = channels[0].at<double>(cv::Point(i, j))*0.299 + channels[1].at<double>(cv::Point(i, j))*0.299 +channels[2].at<double>(cv::Point(i, j))*0.299;
            if(new_frame.at<double>(cv::Point(i, j)) > m_log_threshold)
            {
                new_frame.at<double>(cv::Point(i, j)) = log(new_frame.at<double>(cv::Point(i, j)));
            }
        }
    }
}

void SurroundSuppression::writeEvents(std::vector<Event> &events, float delta_b, float thresh, float frame_id, int x, int y, int polarity)
{
    std::random_device rd;
    std::default_random_engine generator(rd());

    std::uniform_int_distribution<int> distribution(floor(-m_timestamp_noise/2),floor(m_timestamp_noise/2)+1);
    int nb_event;
    int moddiff = int(delta_b/thresh);
    if(moddiff > m_n_max || moddiff <0)
    {
        nb_event = m_n_max;
    }
    else
    {
        nb_event = moddiff;
    }
    for(int e=0; e<nb_event; e++)
    {
        int random_val = distribution(generator);
        int timestamp = ( ( (m_time_gap * (e+1) * thresh) /delta_b) + m_time_gap * frame_id + random_val );
        if(timestamp < 0)
        {
            timestamp = 0;
        }
        events.emplace_back(Event(timestamp,x,y,polarity,false));
        Event ev(timestamp,x,y,polarity,false);
    }
}

void SurroundSuppression::frameToEvents(int frame_id, cv::Mat frame, cv::Mat reference, cv::Mat &threshold_map, std::vector<Event> &events)
{
    cv::Mat delta = frame - reference;
    for(int i=0; i<width_; i++)
    {
        for(int j=0; j<height_; j++)
        {
            if(delta.at<double>(cv::Point(i,j)) > threshold_map.at<double>(cv::Point(i,j)))
            {
                writeEvents(events, delta.at<double>(cv::Point(i,j)), threshold_map.at<double>(cv::Point(i,j)), frame_id, i, j, 1);
                threshold_map.at<double>(cv::Point(i,j)) *= (1+m_adapt_thresh_coef_shift);
            }
            else if(delta.at<double>(cv::Point(i,j)) < -threshold_map.at<double>(cv::Point(i,j)))
            {
                writeEvents(events, -delta.at<double>(cv::Point(i,j)), threshold_map.at<double>(cv::Point(i,j)), frame_id, i, j, 0);
                threshold_map.at<double>(cv::Point(i,j)) *= (1+m_adapt_thresh_coef_shift);
            }
            else if(delta.at<double>(cv::Point(i,j)) <= threshold_map.at<double>(cv::Point(i,j)) || delta.at<double>(cv::Point(i,j)) >= -threshold_map.at<double>(cv::Point(i,j)))
            {
                threshold_map.at<double>(cv::Point(i,j)) *= (1-m_adapt_thresh_coef_shift);
            }
        }
    }
}

void SurroundSuppression::createEvents(const std::string &pathToFrames, std::vector<Event> &events, int nbPass)
{
    cv::Mat threshold_map(cv::Size(width_, height_), CV_64FC1, cv::Scalar(m_map_threshold)); 
    std::vector<std::string> frames;
    for (const auto & frame : std::filesystem::directory_iterator{pathToFrames}) 
    {
        frames.emplace_back(frame.path().filename().stem().string());
    }
    std::sort(frames.begin(), frames.end(), compareImgName);
    std::string extension = ".png";
    cv::Mat init = cv::imread(pathToFrames + frames[0] + extension);
    cv::Mat reference(cv::Size(width_, height_), CV_64FC1);
    convertFrame(init,reference);
    int sze = frames.size();
    int i =1;
    while(i<frames.size())
    {
        std::string newpath = pathToFrames + frames[i] + extension;
        init = cv::imread(pathToFrames + frames[i] + extension);
        if (!init.data) 
        {
            std::cout << "Could not open or find" << " the image" << std::endl;
        }
        cv::Mat frame(cv::Size(width_, height_), CV_64FC1, cv::Scalar(0));
        convertFrame(init,frame);
        
        frameToEvents(i,frame,reference,threshold_map,events);
        reference = frame;
        i++;
    }
    std::sort(events.begin(),events.end());
    int size = events.size();
    long firstTimestamp = static_cast<long>((events.at(0)).timestamp());
    long actualLast = static_cast<long>((events.at(size-1)).timestamp());
    long value = actualLast - firstTimestamp;
    int k=0;
    
    for(i=0; i<nbPass; i++)
    {
        k+=1;
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