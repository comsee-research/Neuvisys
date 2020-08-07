#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>
#include <thread>
#include "src/SpikingNetwork.hpp"

class Neuvisys : public dv::ModuleBase {
private:
    NetworkConfig conf = NetworkConfig(Conf::CONF_FILE);
    dv::EventStreamSlicer slicer;
	SpikingNetwork spinet = SpikingNetwork(conf);
    std::map<std::string, cv::Mat> displays;
    long lastTime;
public:
    Neuvisys() {
        /***** Initialize Network *****/
        lastTime = 0;

        /***** Displays *****/
        outputs.getEventOutput("frames").setup(Conf::WIDTH, Conf::HEIGHT, "frames");
        displays["frames"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);
        outputs.getFrameOutput("potentials").setup(Conf::WIDTH, Conf::HEIGHT, "potentials");
        displays["potentials"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
        outputs.getFrameOutput("spikes").setup(Conf::WIDTH, Conf::HEIGHT, "spikes");
        displays["spikes"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
        outputs.getFrameOutput("weights").setup(20*conf.Neuron1Width, 20*conf.Neuron1Height, "weights");
        displays["weights"] = cv::Mat::zeros(20*conf.Neuron1Height, 20*conf.Neuron1Width, CV_8UC3);
        outputs.getFrameOutput("zoom").setup(conf.Neuron1Width, conf.Neuron1Height, "zoom");
        displays["zoom"] = cv::Mat::zeros(conf.Neuron1Height, conf.Neuron1Width, CV_8UC3);
        outputs.getFrameOutput("potentials2").setup(Conf::WIDTH, Conf::HEIGHT, "potentials2");
        displays["potentials2"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
        outputs.getFrameOutput("weights2").setup(20*conf.Neuron2Width, 20*conf.Neuron2Height, "weights2");
        displays["weights2"] = cv::Mat::zeros(20*conf.Neuron2Height, 20*conf.Neuron2Width, CV_8UC3);
        outputs.getFrameOutput("spikes2").setup(Conf::WIDTH, Conf::HEIGHT, "spikes2");
        displays["spikes2"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);

        /***** Slicers *****/
        slicer.doEveryTimeInterval(Conf::EVENT_FREQUENCY, [this](const dv::EventStore &data) {
            computeEvents(data);
        });
        slicer.doEveryTimeInterval(Conf::DISPLAY_FREQUENCY, [this](const dv::EventStore &data) {
            computeDisplays(data);
        });
        slicer.doEveryTimeInterval(Conf::UPDATE_PARAMETER_FREQUENCY, [this](const dv::EventStore &data) {
            computeParameters();
        });
    }

    static void initInputs(dv::InputDefinitionList &in) {
		in.addEventInput("events");
	}

	static void initOutputs(dv::OutputDefinitionList &out) {
        out.addFrameOutput("frames");
        out.addFrameOutput("potentials");
        out.addFrameOutput("spikes");
        out.addFrameOutput("weights");
        out.addFrameOutput("zoom");
        out.addFrameOutput("weights2");
        out.addFrameOutput("potentials2");
        out.addFrameOutput("spikes2");
	}

    void computeDisplays(const dv::EventStore &events) {
        spinet.updateDisplay(lastTime, displays);

        auto frame = outputs.getFrameOutput("frames").frame();
        frame << displays["frames"];
        frame.commit();
        displays["frames"] = 0;
        auto potentials = outputs.getFrameOutput("potentials").frame();
        potentials << displays["potentials"];
        potentials.commit();
        auto spikes = outputs.getFrameOutput("spikes").frame();
        spikes << displays["spikes"];
        spikes.commit();
        auto weights = outputs.getFrameOutput("weights").frame();
        weights.setFormat(dv::FrameFormat::BGR);
        weights << displays["weights"];
        weights.commit();
        auto zoom = outputs.getFrameOutput("zoom").frame();
        weights.setFormat(dv::FrameFormat::BGR);
        zoom << displays["zoom"];
        zoom.commit();
        auto weights2 = outputs.getFrameOutput("weights2").frame();
        weights2.setFormat(dv::FrameFormat::BGR);
        weights2 << displays["weights2"];
        weights2.commit();
        auto potentials2 = outputs.getFrameOutput("potentials2").frame();
        potentials2 << displays["potentials2"];
        potentials2.commit();
        auto spikes2 = outputs.getFrameOutput("spikes2").frame();
        spikes2 << displays["spikes2"];
        spikes2.commit();
    }

	void computeEvents(const dv::EventStore &events) {
        if (!events.isEmpty()) {
            lastTime = events.getHighestTime();
            for (const dv::Event &event : events) {
                spinet.addEvent(event.timestamp(), event.x(), event.y(), event.polarity());

                displays["frames"].at<cv::Vec3b>(event.y(), event.x())[2-event.polarity()] = 255;
            }
            spinet.updateNeurons(lastTime);
        }
    }

    void computeParameters() {
        spinet.updateNeuronsParameters(lastTime);
        json gui;

        std::ifstream ifs(Conf::GUI_FILE);
        if (ifs.is_open()) {
            try {
                ifs >> gui;
            } catch (const std::exception& e) {
                std::cerr << "In GUI config file" << std::endl;
                throw;
            }
        } else {
            std::cout << "cannot open GUI file" << std::endl;
        }
        ifs.close();

        Selection::INDEX = gui["index"];
        Selection::INDEX2 = gui["index2"];
        Selection::LAYER = gui["layer"];
        Selection::LAYER2 = gui["layer2"];
        if (gui["save"]) {
            spinet.saveWeights();
            gui["save"] = false;

            std::ofstream ofs(Conf::GUI_FILE);
            if (ofs.is_open()) {
                ofs << std::setw(4) << gui << std::endl;
            } else {
                std::cout << "cannot open GUI file" << std::endl;
            }
            ofs.close();
        }

        if (Selection::INDEX > spinet.getNumberNeurons()) {
            std::cout << "neuron display index too big" << std::endl;
            Selection::INDEX = 0;
        }
        if (Selection::INDEX2 > spinet.getNumberPoolingNeurons()) {
            std::cout << "pooling neuron display index too big" << std::endl;
            Selection::INDEX2 = 0;
        }
        if (Selection::LAYER > conf.L1Depth) {
            std::cout << "neuron display layer too big" << std::endl;
            Selection::LAYER = 0;
        }

        config.setLong("spiking_rate", static_cast<long>(1000 * spinet.getNeuron(Selection::INDEX).getSpikingRate()));
        config.setLong("threshold", static_cast<long>(spinet.getNeuron(Selection::INDEX).getThreshold()));
        config.setLong("adaptation_potential", static_cast<long>(1000 * spinet.getNeuron(Selection::INDEX).getAdaptationPotential()));
        config.setLong("learning_decay", static_cast<long>(100 * spinet.getNeuron(Selection::INDEX).getLearningDecay()));
    }

	void run() override {
        slicer.accept(inputs.getEventInput("events").events());
    }

    static const char *initDescription() {
        return ("Neuvisys module.");
    }

    static void initConfigOptions(dv::RuntimeConfig &config) {
        config.add("spiking_rate", dv::ConfigOption::statisticOption("Spiking Rate"));
        config.add("threshold", dv::ConfigOption::statisticOption("Threshold"));
        config.add("adaptation_potential", dv::ConfigOption::statisticOption("Adaptation Potential"));
        config.add("learning_decay", dv::ConfigOption::statisticOption("Learning Decay"));
    }

    void configUpdate() override {
    }
};

registerModuleClass(Neuvisys)
