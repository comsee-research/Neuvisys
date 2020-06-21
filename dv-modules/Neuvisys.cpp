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
        if (conf.SAVE_DATA) {
            spinet.loadWeights();
        }
        lastTime = 0;

        /***** Displays *****/
        outputs.getEventOutput("frames").setup(Conf::WIDTH, Conf::HEIGHT, "frames");
        displays["frames"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3);
        outputs.getFrameOutput("potentials").setup(Conf::WIDTH, Conf::HEIGHT, "potentials");
        displays["potentials"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
        outputs.getFrameOutput("spikes").setup(Conf::WIDTH, Conf::HEIGHT, "spikes");
        displays["spikes"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
        outputs.getFrameOutput("weights").setup(conf.Neuron1Width, conf.Neuron1Height, "weights");
        displays["weights"] = cv::Mat::zeros(conf.Neuron1Height, conf.Neuron1Width, CV_8UC3);
        outputs.getFrameOutput("zoom").setup(conf.Neuron1Width, conf.Neuron1Height, "zoom");
        displays["zoom"] = cv::Mat::zeros(conf.Neuron1Height, conf.Neuron1Width, CV_8UC3);
        outputs.getFrameOutput("potentials2").setup(Conf::WIDTH, Conf::HEIGHT, "potentials2");
        displays["potentials2"] = cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1);
        outputs.getFrameOutput("weights2").setup(conf.Neuron2Width, conf.Neuron2Height, "weights2");
        displays["weights2"] = cv::Mat::zeros(conf.Neuron2Height, conf.Neuron2Width, CV_8UC3);
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

    ~Neuvisys() override {
        std::cout << "Network reset" << std::endl;
        if (conf.SAVE_DATA) {
            spinet.saveWeights();
        }
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
        }
        spinet.updateNeurons(lastTime);
    }

    void computeParameters() {
        spinet.updateNeuronsParameters();

        config.setLong("spiking_rate", static_cast<long>(1000 * spinet.getNeuron(Selection::IND).getSpikingRate()));
        config.setLong("threshold", static_cast<long>(spinet.getNeuron(Selection::IND).getThreshold()));
        config.setLong("adaptation_potential", static_cast<long>(1000 * spinet.getNeuron(Selection::IND).getAdaptationPotential()));
        config.setLong("learning_decay", static_cast<long>(100 * spinet.getNeuron(Selection::IND).getLearningDecay()));
    }

	void run() override {
        slicer.accept(inputs.getEventInput("events").events());
    }

    static const char *initDescription() {
        return ("Neuvisys module.");
    }

    static void initConfigOptions(dv::RuntimeConfig &config) {
        Selection::X_NEURON = 0;
        Selection::Y_NEURON = 0;
        Selection::LAYER = 0;
        Selection::IND = static_cast<unsigned int>(Selection::X_NEURON * Selection::NET_HEIGHT * Selection::NET_DEPTH +
                                                   Selection::Y_NEURON * Selection::NET_DEPTH + Selection::LAYER);

        config.add("X_NEURON", dv::ConfigOption::intOption("X Position of the neuron to display", Selection::X_NEURON, 0, Selection::NET_WIDTH - 1));
        config.add("Y_NEURON", dv::ConfigOption::intOption("Y Position of the neuron to display", Selection::Y_NEURON, 0, Selection::NET_HEIGHT - 1));
        config.add("SYNAPSE", dv::ConfigOption::intOption("Layer of the neuron to display", Selection::LAYER, 0, Selection::NET_SYNAPSES - 1));
        config.add("LAYER", dv::ConfigOption::intOption("Layer of the neuron to display", Selection::LAYER, 0, Selection::NET_DEPTH - 1));

        config.add("spiking_rate", dv::ConfigOption::statisticOption("Spiking Rate"));
        config.add("threshold", dv::ConfigOption::statisticOption("Threshold"));
        config.add("adaptation_potential", dv::ConfigOption::statisticOption("Adaptation Potential"));
        config.add("learning_decay", dv::ConfigOption::statisticOption("Learning Decay"));
    }

    void configUpdate() override {
        Selection::X_NEURON = config.getInt("X_NEURON");
        Selection::Y_NEURON = config.getInt("Y_NEURON");
        Selection::SYNAPSE = config.getInt("SYNAPSE");
        Selection::LAYER = config.getInt("LAYER");
        Selection::IND = static_cast<unsigned int>(Selection::X_NEURON * conf.L1Height * conf.L1Depth +
                                                   Selection::Y_NEURON * conf.L1Depth + Selection::LAYER);
    }
};

registerModuleClass(Neuvisys)
