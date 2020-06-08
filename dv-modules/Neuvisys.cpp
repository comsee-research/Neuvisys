#include <dv-sdk/module.hpp>
#include <dv-sdk/processing/core.hpp>
#include <thread>
#include "src/SpikingNetwork.hpp"

class Neuvisys : public dv::ModuleBase {
private:
    NetworkConfig conf = NetworkConfig(Conf::CONF_FILE);
    dv::EventStreamSlicer slicer;
	SpikingNetwork spinet = SpikingNetwork(conf);
    std::vector<cv::Mat> displays;
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
        displays.emplace_back(cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC3));

        outputs.getFrameOutput("potentials").setup(Conf::WIDTH, Conf::HEIGHT, "potentials");
        displays.emplace_back(cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1));

        outputs.getFrameOutput("spikes").setup(Conf::WIDTH, Conf::HEIGHT, "spikes");
        displays.emplace_back(cv::Mat::zeros(Conf::HEIGHT, Conf::WIDTH, CV_8UC1));

        outputs.getFrameOutput("weights").setup(conf.NEURON_WIDTH, conf.NEURON_HEIGHT, "weights");
        displays.emplace_back(cv::Mat::zeros(conf.NEURON_HEIGHT, conf.NEURON_WIDTH, CV_8UC3));

        outputs.getFrameOutput("zoom").setup(conf.NEURON_WIDTH, conf.NEURON_HEIGHT, "zoom");
        displays.emplace_back(cv::Mat::zeros(conf.NEURON_HEIGHT, conf.NEURON_WIDTH, CV_8UC3));

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
	}

    void computeDisplays(const dv::EventStore &events) {
        spinet.updateDisplay(lastTime, displays);

        auto frame = outputs.getFrameOutput("frames").frame();
        frame << displays[0];
        frame.commit();
        displays[0] = 0;

        outputs.getFrameOutput("potentials").frame() << displays[1];
        outputs.getFrameOutput("potentials").frame().commit();

        outputs.getFrameOutput("spikes").frame() << displays[2];
        outputs.getFrameOutput("spikes").frame().commit();

        auto weights = outputs.getFrameOutput("weights").frame();
        weights.setFormat(dv::FrameFormat::BGR);
        weights << displays[3];
        weights.commit();

        auto zoom = outputs.getFrameOutput("zoom").frame();
        weights.setFormat(dv::FrameFormat::BGR);
        zoom << displays[4];
        zoom.commit();
    }

	void computeEvents(const dv::EventStore &events) {
        if (!events.isEmpty()) {
            lastTime = events.getHighestTime();
            for (const dv::Event &event : events) {
                spinet.addEvent(event.timestamp(), event.x(), event.y(), event.polarity());

                displays[0].at<cv::Vec3b>(event.y(), event.x())[2-event.polarity()] = 255;
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
        Selection::IND = Selection::X_NEURON * 24 * 10 + Selection::Y_NEURON * 10 + Selection::LAYER;

        config.add("X_NEURON", dv::ConfigOption::intOption("X Position of the neuron to display", Selection::X_NEURON, 0, 36 - 1));
        config.add("Y_NEURON", dv::ConfigOption::intOption("Y Position of the neuron to display", Selection::Y_NEURON, 0, 24 - 1));
        config.add("SYNAPSE", dv::ConfigOption::intOption("Layer of the neuron to display", Selection::LAYER, 0, 1 - 1));
        config.add("LAYER", dv::ConfigOption::intOption("Layer of the neuron to display", Selection::LAYER, 0, 10 - 1));

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
        Selection::IND = Selection::X_NEURON * conf.NETWORK_HEIGHT * conf.NETWORK_DEPTH + Selection::Y_NEURON * conf.NETWORK_DEPTH + Selection::LAYER;
    }
};

registerModuleClass(Neuvisys)
