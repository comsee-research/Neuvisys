//
// Created by Thomas on 14/04/2021.
//

#include "NeuvisysguiSimulation.h"
#include "ui_neuvisysguisimulation.h"

NeuvisysGUISimulation::NeuvisysGUISimulation(int argc, char **argv, const std::string &eventPath, const std::string &networkPath, QWidget *parent)
        : QMainWindow(parent),
          neuvisysThread(argc, argv),
          ui(new Ui::NeuvisysGUISimulation) {
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(3, 2);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);

    precisionEvent = 5000;
    precisionPotential = 10000;
    rangePotential = 1000000;
    rangeSpiketrain = 1000000;

    ui->setupUi(this);
    ui->text_event_file->setText(QString::fromStdString(eventPath));
    ui->text_network_directory->setText(QString::fromStdString(networkPath));
    openConfigFiles();
    ui->number_runs->setValue(1);
    ui->progressBar->setValue(0);
    ui->modeChoice->setId(ui->recording, 0);
    ui->modeChoice->setId(ui->realtime, 1);
    ui->modeChoice->setId(ui->simulation, 2);
    ui->modeChoice->setId(ui->events_only, 3);
    buttonSelectionGroup = new QButtonGroup(this);

    /* Selection parameters */
    ui->slider_precision_event->setMaximum(100000);
    ui->slider_precision_event->setMinimum(5000);
    ui->slider_precision_event->setValue(static_cast<int>(precisionEvent));

    ui->slider_precision_potential->setMaximum(100000);
    ui->slider_precision_potential->setMinimum(500);
    ui->slider_precision_potential->setValue(static_cast<int>(precisionPotential));
    ui->slider_range_potential->setMaximum(5000000);
    ui->slider_range_potential->setMinimum(1000);
    ui->slider_range_potential->setValue(static_cast<int>(rangePotential));
    ui->slider_range_spiketrain->setMaximum(10000000);
    ui->slider_range_spiketrain->setMinimum(100000);
    ui->slider_range_spiketrain->setValue(static_cast<int>(rangeSpiketrain));

    ui->lcd_precision_event->display(static_cast<double>(precisionEvent) / 1000);
    ui->lcd_precision_potential->display(static_cast<double>(precisionPotential) / 1000);
    ui->lcd_range_potential->display(static_cast<double>(rangePotential) / 1000);
    ui->lcd_range_spiketrain->display(static_cast<double>(rangeSpiketrain) / 1000);

    /* Qt charts */
    eventRateSeries = new QLineSeries();
    networkRateSeries = new QLineSeries();
    eventRateChart = new QChart();
    networkRateChart = new QChart();
    eventRateChart->legend()->hide();
    eventRateChart->addSeries(eventRateSeries);
    eventRateChart->createDefaultAxes();
    eventRateChart->setTitle("Event spike rates");
    networkRateChart->legend()->hide();
    networkRateChart->addSeries(networkRateSeries);
    networkRateChart->createDefaultAxes();
    networkRateChart->setTitle("Network spike rates");
    ui->eventRateView->setChart(eventRateChart);
    ui->eventRateView->setRenderHint(QPainter::Antialiasing);
    ui->networkRateView->setChart(networkRateChart);
    ui->networkRateView->setRenderHint(QPainter::Antialiasing);

    potentialSeries = new QLineSeries();
    potentialChart = new QChart();
    potentialChart->legend()->hide();
    potentialChart->addSeries(potentialSeries);
    potentialChart->createDefaultAxes();
    potentialChart->setTitle("Potential train of selected neuron");
    ui->potentialView->setChart(potentialChart);
    ui->potentialView->setRenderHint(QPainter::Antialiasing);

    spikeSeries = new QScatterSeries();
    spikeSeries->setMarkerSize(1.0);
    spikeSeries->setMarkerShape(QScatterSeries::MarkerShapeRectangle);
    spikeChart = new QChart();
    spikeChart->legend()->hide();
    spikeChart->addSeries(spikeSeries);
    spikeChart->createDefaultAxes();
    spikeChart->setTitle("Spike train of selected layer");
    ui->spikeView->setChart(spikeChart);
    ui->spikeView->setRenderHint(QPainter::Antialiasing);

    rewardSeries = new QLineSeries();
    valueSeries = new QLineSeries();
    valueDotSeries = new QLineSeries();
    tdSeries = new QLineSeries();
    rewardChart = new QChart();
    rewardChart->legend()->hide();
    rewardChart->addSeries(rewardSeries);
    rewardChart->addSeries(valueSeries);
    rewardChart->addSeries(valueDotSeries);
    rewardChart->addSeries(tdSeries);
    rewardChart->createDefaultAxes();
    rewardChart->setTitle("Reward plot");
    ui->rewardView->setChart(rewardChart);
    ui->rewardView->setRenderHint(QPainter::Antialiasing);

    actionChart = new QChart();
    actionChart->legend()->hide();
    actionChart->createDefaultAxes();
    actionChart->setTitle("Action plot");
    ui->actionView->setChart(actionChart);
    ui->actionView->setRenderHint(QPainter::Antialiasing);
}

NeuvisysGUISimulation::~NeuvisysGUISimulation() {
    delete ui;
    free(potentialSeries);
    free(potentialChart);
    free(spikeSeries);
    free(spikeChart);
    free(rewardSeries);
    free(valueSeries);
    free(rewardChart);
}

void NeuvisysGUISimulation::on_button_launch_network_clicked() {
    qRegisterMetaType<size_t>("size_t");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<std::vector<double>>("std::vector<double>");
    qRegisterMetaType<std::vector<bool>>("std::vector<bool>");
    qRegisterMetaType<std::vector<size_t>>("std::vector<size_t>");
    qRegisterMetaType<std::map<size_t, cv::Mat>>("std::map<size_t, cv::Mat>");
    qRegisterMetaType<std::vector<std::reference_wrapper<const std::vector<size_t>>>>(
            "std::vector<std::reference_wrapper<const std::vector<size_t>>>");
    qRegisterMetaType<std::vector<std::vector<size_t>>>("std::vector<std::vector<size_t>>");
    qRegisterMetaType<std::vector<std::vector<size_t>>>("std::vector<std::vector<size_t>>");
    qRegisterMetaType<std::vector<std::vector<double>>>("std::vector<std::vector<double>>");
    qRegisterMetaType<std::vector<std::pair<double, size_t>>>("std::vector<std::pair<double, size_t>>");

    connect(&neuvisysThread, &NeuvisysThreadSimulation::displayProgress, this, &NeuvisysGUISimulation::onDisplayProgress);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::displayStatistics, this, &NeuvisysGUISimulation::onDisplayStatistics);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::displayEvents, this, &NeuvisysGUISimulation::onDisplayEvents);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::displayPotential, this, &NeuvisysGUISimulation::onDisplayPotential);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::displaySpike, this, &NeuvisysGUISimulation::onDisplaySpike);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::displayWeights, this, &NeuvisysGUISimulation::onDisplayWeights);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::displayReward, this, &NeuvisysGUISimulation::onDisplayReward);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::displayAction, this, &NeuvisysGUISimulation::onDisplayAction);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::networkConfiguration, this, &NeuvisysGUISimulation::onNetworkConfiguration);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::networkCreation, this, &NeuvisysGUISimulation::onNetworkCreation);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::networkDestruction, this, &NeuvisysGUISimulation::onNetworkDestruction);
    connect(&neuvisysThread, &NeuvisysThreadSimulation::consoleMessage, this, &NeuvisysGUISimulation::onConsoleMessage);
    neuvisysThread.init();

    connect(this, &NeuvisysGUISimulation::tabVizChanged, &neuvisysThread, &NeuvisysThreadSimulation::onTabVizChanged);
    connect(this, &NeuvisysGUISimulation::indexChanged, &neuvisysThread, &NeuvisysThreadSimulation::onIndexChanged);
    connect(this, &NeuvisysGUISimulation::zcellChanged, &neuvisysThread, &NeuvisysThreadSimulation::onZcellChanged);
    connect(this, &NeuvisysGUISimulation::cameraChanged, &neuvisysThread, &NeuvisysThreadSimulation::onCameraChanged);
    connect(this, &NeuvisysGUISimulation::synapseChanged, &neuvisysThread, &NeuvisysThreadSimulation::onSynapseChanged);
    connect(this, &NeuvisysGUISimulation::precisionEventChanged, &neuvisysThread, &NeuvisysThreadSimulation::onPrecisionEventChanged);
    connect(this, &NeuvisysGUISimulation::rangePotentialChanged, &neuvisysThread, &NeuvisysThreadSimulation::onRangePotentialChanged);
    connect(this, &NeuvisysGUISimulation::precisionPotentialChanged, &neuvisysThread,
            &NeuvisysThreadSimulation::onPrecisionPotentialChanged);
    connect(this, &NeuvisysGUISimulation::rangeSpikeTrainChanged, &neuvisysThread, &NeuvisysThreadSimulation::onRangeSpikeTrainChanged);
    connect(this, &NeuvisysGUISimulation::layerChanged, &neuvisysThread, &NeuvisysThreadSimulation::onLayerChanged);
    connect(this, &NeuvisysGUISimulation::stopNetwork, &neuvisysThread, &NeuvisysThreadSimulation::onStopNetwork);

    neuvisysThread.render(ui->text_network_directory->text() + "/",
                          ui->text_event_file->text(),
                          static_cast<size_t>(ui->number_runs->value()), ui->modeChoice->checkedId());
    ui->console->insertPlainText(QString("Starting network...\n"));
}

void NeuvisysGUISimulation::on_text_network_directory_textChanged() {
    openConfigFiles();
}

void NeuvisysGUISimulation::on_text_network_config_textChanged() {
    QString confDir = ui->text_network_directory->text() + "/configs/network_config.json";
    QString text = ui->text_network_config->toPlainText();
    modifyConfFile(confDir, text);
}

void NeuvisysGUISimulation::on_text_rl_config_textChanged() {
    QString confDir = ui->text_network_directory->text() + "/configs/rl_config.json";
    QString text = ui->text_rl_config->toPlainText();
    modifyConfFile(confDir, text);
}

void NeuvisysGUISimulation::on_text_simple_cell_config_textChanged() {
    QString confDir = ui->text_network_directory->text() + "/configs/simple_cell_config.json";
    QString text = ui->text_simple_cell_config->toPlainText();
    modifyConfFile(confDir, text);
}

void NeuvisysGUISimulation::on_text_complex_cell_config_textChanged() {
    QString confDir = ui->text_network_directory->text() + "/configs/complex_cell_config.json";
    QString text = ui->text_complex_cell_config->toPlainText();
    modifyConfFile(confDir, text);
}

void NeuvisysGUISimulation::on_text_critic_cell_config_textChanged() {
    QString confDir = ui->text_network_directory->text() + "/configs/critic_cell_config.json";
    QString text = ui->text_critic_cell_config->toPlainText();
    modifyConfFile(confDir, text);
}

void NeuvisysGUISimulation::on_text_actor_cell_config_textChanged() {
    QString confDir = ui->text_network_directory->text() + "/configs/actor_cell_config.json";
    QString text = ui->text_actor_cell_config->toPlainText();
    modifyConfFile(confDir, text);
}

void NeuvisysGUISimulation::on_button_event_file_clicked() {
    QString fileName = QFileDialog::getOpenFileName(this, "Open the file");
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    ui->text_event_file->setText(fileName);
    file.close();
}

void NeuvisysGUISimulation::on_button_network_directory_clicked() {
    QString dir = QFileDialog::getExistingDirectory(this, "Open Directory", "/home",
                                                    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    ui->text_network_directory->setText(dir);
    openConfigFiles();
}

void NeuvisysGUISimulation::on_button_create_network_clicked() {
    QString dir = QFileDialog::getSaveFileName(this, "Open Directory", "/home");
    NetworkConfig::createNetwork(dir.toStdString(), PredefinedConfigurations::fourLayerRLOnePatchCenteredConfig);
    ui->text_network_directory->setText(dir);
    openConfigFiles();
}

void NeuvisysGUISimulation::on_button_stop_network_clicked() {
    emit stopNetwork();
    ui->console->insertPlainText(QString("Saving network...\n"));
}

void NeuvisysGUISimulation::openConfigFiles() {
    QString dir = ui->text_network_directory->text();
    QString confDir = dir + "/configs/network_config.json";
    ui->text_network_config->setText(readConfFile(confDir));
    confDir = dir + "/configs/rl_config.json";
    ui->text_rl_config->setText(readConfFile(confDir));
    confDir = dir + "/configs/simple_cell_config.json";
    ui->text_simple_cell_config->setText(readConfFile(confDir));
    confDir = dir + "/configs/complex_cell_config.json";
    ui->text_complex_cell_config->setText(readConfFile(confDir));
    confDir = dir + "/configs/critic_cell_config.json";
    ui->text_critic_cell_config->setText(readConfFile(confDir));
    confDir = dir + "/configs/actor_cell_config.json";
    ui->text_actor_cell_config->setText(readConfFile(confDir));
}

QString NeuvisysGUISimulation::readConfFile(QString &directory) {
    QFileInfo checkFile(directory);
    if (checkFile.exists() && checkFile.isFile()) {
        QFile file(directory);
        if (!file.open(QIODevice::ReadOnly | QFile::Text)) {
            QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        }
        QString networkText = QTextStream(&file).readAll();
        file.close();
        return networkText;
    } else {
        return {"File Not Found !"};
    }
}

void NeuvisysGUISimulation::modifyConfFile(QString &directory, QString &text) {
    QFileInfo checkFile(directory);
    if (checkFile.exists() && checkFile.isFile()) {
        QFile file(directory);
        if (!file.open(QIODevice::WriteOnly | QFile::Text)) {
            std::cout << "Warning:" << directory.toStdString() << "file does not exist !" << std::endl;
        } else {
            QTextStream out(&file);
            out << text;
        }
        file.close();
    }
}

void NeuvisysGUISimulation::onNetworkCreation(const size_t nbCameras, const size_t nbSynapses,
                                              const std::vector<size_t> &networkStructure,
                                              const size_t vfWidth, const size_t vfHeight,
                                              const size_t nbActions) {
    ui->console->clear();
    ui->spin_camera_selection->setMaximum(static_cast<int>(nbCameras - 1));
    ui->spin_synapse_selection->setMaximum(static_cast<int>(nbSynapses - 1));
    ui->slider_layer->setMaximum(static_cast<int>(networkStructure.size() - 1));

    ui->console->insertPlainText(QString("Network started:\n"));
    QString message = QString("Network structure: ");
    for (auto structure: networkStructure) {
        message.append(QString::number(structure));
        message.append(QString(" / "));
    }
    message.append(QString("\n"));
    ui->console->insertPlainText(message);

    m_vfWidth = vfWidth;
    m_vfHeight = vfHeight;

    for (int i = 0; i < nbActions; ++i) {
        actionSeries.push_back(new QLineSeries());
        actionSeries[i]->setName(QString::fromStdString("Action " + std::to_string(i)));
        actionChart->addSeries(actionSeries[i]);
    }
    actionChart->legend()->hide();
    actionChart->createDefaultAxes();
    actionChart->setTitle("Action plot");
    ui->actionView->setChart(actionChart);
    ui->actionView->setRenderHint(QPainter::Antialiasing);
}

void NeuvisysGUISimulation::onNetworkConfiguration(const std::vector<std::vector<size_t>> &layerPatches,
                                                   const std::vector<size_t> &layerSizes, const std::vector<size_t> &neuronSizes) {
    ui->spin_zcell_selection->setMaximum(static_cast<int>(layerSizes[2] - 1));
    ui->spin_zcell_selection->setValue(0);
    buttonSelectionGroup = new QButtonGroup;

    while (QLayoutItem *item = ui->gridSelection->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    while (QLayoutItem *item = ui->weightLayout->takeAt(0)) {
        delete item->widget();
        delete item;
    }

    weightLayoutPreparation(layerPatches, layerSizes);

    int count = 0;
    for (size_t i = 0; i < layerPatches[0].size() * layerSizes[0]; ++i) {
        for (size_t j = 0; j < layerPatches[1].size() * layerSizes[1]; ++j) {
            auto *button = new QPushButton(this);
            button->setText(QString::number(count));
            button->setFixedWidth(30);
            button->setFixedHeight(30);
            button->setCheckable(true);
            if (count == 0) {
                button->setChecked(true);
            }
            buttonSelectionGroup->addButton(button);
            buttonSelectionGroup->setId(button, count);
            ui->gridSelection->addWidget(button, static_cast<int>(j), static_cast<int>(i));
            button->show();
            ++count;
        }
    }
    connect(buttonSelectionGroup, SIGNAL(buttonClicked(int)), this, SLOT(on_button_selection_clicked(int)));

}

void NeuvisysGUISimulation::weightLayoutPreparation(const std::vector<std::vector<size_t>> &layerPatches,
                                                    const std::vector<size_t> &layerSizes) {
    if (m_layer == 0) {
        for (size_t wp = 0; wp < layerPatches[0].size(); ++wp) {
            for (size_t hp = 0; hp < layerPatches[1].size(); ++hp) {
                for (size_t i = 0; i < static_cast<size_t>(std::sqrt(layerSizes[2])); ++i) {
                    for (size_t j = 0; j < static_cast<size_t>(std::sqrt(layerSizes[2])); ++j) {
                        auto *label = new QLabel(this);
                        ui->weightLayout->addWidget(label, static_cast<int>(hp * layerSizes[2] + i),
                                                    static_cast<int>(wp * layerSizes[2] + j));
                        label->show();
                    }
                }
            }
        }
    } else if (m_layer == 1) {
        for (size_t i = 0; i < layerPatches[0].size() * layerSizes[0]; ++i) {
            for (size_t j = 0; j < layerPatches[1].size() * layerSizes[1]; ++j) {
                auto *label = new QLabel(this);
                ui->weightLayout->addWidget(label, static_cast<int>(j), static_cast<int>(i));
                label->show();
            }
        }
    } else if (m_layer == 2) {
        auto *label = new QLabel(this);
        ui->weightLayout->addWidget(label, 0, 0);
        label->show();
    } else if (m_layer == 3) {
        for (size_t i = 0; i < actionSeries.size(); ++i) {
            auto *label = new QLabel(this);
            ui->weightLayout->addWidget(label, static_cast<int>(i), 0);
            label->show();
        }
    }
}

void NeuvisysGUISimulation::on_button_selection_clicked(int index) {
    m_id = static_cast<size_t>(ui->spin_zcell_selection->value() + 1) * index;
    emit indexChanged(m_id);
}

void NeuvisysGUISimulation::on_spin_zcell_selection_valueChanged(int arg1) {
    m_zcell = static_cast<size_t>(arg1);
    m_id = static_cast<size_t>(m_zcell + 1) * buttonSelectionGroup->checkedId();
    emit zcellChanged(m_zcell);
    emit indexChanged(m_id);
}

void NeuvisysGUISimulation::on_spin_camera_selection_valueChanged(int arg1) {
    m_camera = static_cast<size_t>(arg1);
    emit cameraChanged(m_camera);
}

void NeuvisysGUISimulation::on_spin_synapse_selection_valueChanged(int arg1) {
    m_synapse = static_cast<size_t>(arg1);
    emit synapseChanged(m_synapse);
}

void NeuvisysGUISimulation::on_tab_visualization_currentChanged(int index) {
    emit tabVizChanged(index);
}

void NeuvisysGUISimulation::onDisplayProgress(int progress, double time) {
    ui->progressBar->setValue(progress);
    ui->lcd_sim_time->display(time);
}

void NeuvisysGUISimulation::onDisplayStatistics(const std::vector<double> &eventRateTrain, const std::vector<double> &networkRateTrain) {
    eventRateChart->removeSeries(eventRateSeries);
    networkRateChart->removeSeries(networkRateSeries);
    eventRateSeries = new QLineSeries();
    networkRateSeries = new QLineSeries();

    auto end = eventRateTrain.size();
    for (auto i = 1; i < 200; ++i) {
        if (i >= end) { break; }
        eventRateSeries->append(static_cast<qreal>(end - i), eventRateTrain[end - i]);
        networkRateSeries->append(static_cast<qreal>(end - i), networkRateTrain[end - i]);
    }
    eventRateChart->addSeries(eventRateSeries);
    networkRateChart->addSeries(networkRateSeries);
    networkRateSeries->setColor(QColor(Qt::green));

    eventRateChart->createDefaultAxes();
    networkRateChart->createDefaultAxes();

    ui->eventRateView->repaint();
    ui->networkRateView->repaint();
}

void NeuvisysGUISimulation::onDisplayEvents(const cv::Mat &leftEventDisplay, const cv::Mat &rightEventDisplay) {
    m_leftImage = QImage(static_cast<const uchar *>(leftEventDisplay.data), leftEventDisplay.cols, leftEventDisplay.rows, static_cast<int>
    (leftEventDisplay.step), QImage::Format_RGB888).rgbSwapped().scaled(static_cast<int>(1.5 * m_vfWidth), static_cast<int>(1.5 * m_vfHeight));
    m_rightImage = QImage(static_cast<const uchar *>(rightEventDisplay.data), rightEventDisplay.cols, rightEventDisplay.rows, static_cast<int>
    (rightEventDisplay.step), QImage::Format_RGB888).rgbSwapped().scaled(static_cast<int>(1.5 * m_vfWidth), static_cast<int>(1.5 * m_vfHeight));
    ui->opengl_left_events->setImage(m_leftImage);
    ui->opengl_left_events->update();
    ui->opengl_right_events->setImage(m_rightImage);
    ui->opengl_right_events->update();
}

void NeuvisysGUISimulation::onDisplayWeights(const std::map<size_t, cv::Mat> &weightDisplay) {
    int count = 0;
    for (auto &weight: weightDisplay) {
        QImage weightImage(static_cast<const uchar *>(weight.second.data), weight.second.cols, weight.second.rows,
                           static_cast<int>(weight.second.step), QImage::Format_RGB888);
        if (count < ui->weightLayout->count()) {
            auto label = dynamic_cast<QLabel *>(ui->weightLayout->itemAt(count)->widget());
            if (m_layer == 0) {
                label->setPixmap(QPixmap::fromImage(weightImage.rgbSwapped()).scaled(40, 40, Qt::KeepAspectRatio));
            } else if (m_layer == 1) {
                label->setPixmap(QPixmap::fromImage(weightImage.rgbSwapped()).scaled(400, 400, Qt::KeepAspectRatio));
            } else if (m_layer > 1) {
                label->setPixmap(QPixmap::fromImage(weightImage.rgbSwapped()).scaled(400, 400, Qt::KeepAspectRatio));
            }
        }
        ++count;
    }
}

void NeuvisysGUISimulation::onDisplayPotential(double spikeRate, double vreset, double threshold,
                                               const std::vector<std::pair<double, size_t>> &potentialTrain) {
    potentialChart->removeSeries(potentialSeries);
    potentialSeries = new QLineSeries();
    size_t last = 0;
    if (!potentialTrain.empty()) {
        last = potentialTrain.back().second;
    }
    for (auto it = potentialTrain.rbegin(); it != potentialTrain.rend(); ++it) {
        potentialSeries->append(static_cast<qreal>(it->second), it->first);
        if (last - it->second > static_cast<size_t>(rangePotential)) {
            break;
        }
    }
    potentialChart->addSeries(potentialSeries);
    potentialChart->createDefaultAxes();
    potentialChart->axes(Qt::Vertical, potentialSeries)[0]->setRange(vreset, threshold);
    ui->potentialView->repaint();

    ui->lcd_spike_rate->display(spikeRate);
    ui->lcd_threshold->display(threshold);
    ui->lcd_reset->display(vreset);
}

void NeuvisysGUISimulation::onDisplaySpike(const std::vector<std::reference_wrapper<const std::vector<size_t>>> &spikeTrains, double time) {
    spikeChart->removeSeries(spikeSeries);
    spikeSeries = new QScatterSeries();
    spikeSeries->setMarkerShape(QScatterSeries::MarkerShapeRectangle);
    spikeSeries->setMarkerSize(8);

    size_t index = 0;
    size_t spikeLimit = spikeTrains.size();
    size_t nMax = 1000 / spikeTrains.size();
    if (nMax < 1) {
        nMax = 10;
        spikeLimit = 1000;
    }
    for (const auto &spikeTrain: spikeTrains) {
        size_t count = 0;
        for (auto spike = spikeTrain.get().rbegin(); spike != spikeTrain.get().rend(); ++spike) {
            if (time - static_cast<double>(*spike) <= static_cast<double>(rangeSpiketrain) && count < nMax) {
                spikeSeries->append(static_cast<qreal>(*spike), static_cast<qreal>(index));
                ++count;
            } else {
                break;
            }
        }
        if (index > spikeLimit) {
            break;
        }
        ++index;
    }
    spikeChart->addSeries(spikeSeries);
    spikeChart->createDefaultAxes();
    spikeChart->axes(Qt::Vertical, spikeSeries)[0]->setRange(0, static_cast<int>(spikeLimit));
    ui->spikeView->repaint();
}

void NeuvisysGUISimulation::onDisplayReward(const std::vector<double> &rewardTrain, const std::vector<double> &valueTrain,
                                            const std::vector<double> &valueDotTrain, const std::vector<double> &tdTrain) {
    rewardChart->removeSeries(rewardSeries);
    rewardChart->removeSeries(valueSeries);
    rewardChart->removeSeries(valueDotSeries);
    rewardChart->removeSeries(tdSeries);
    rewardSeries = new QLineSeries();
    rewardSeries->setName("Reward");
    valueSeries = new QLineSeries();
    valueSeries->setName("Value");
    valueDotSeries = new QLineSeries();
    valueDotSeries->setName("Value Derivative");
    tdSeries = new QLineSeries();
    tdSeries->setName("TD Error");

    auto end = rewardTrain.size();
    for (auto i = 1; i < 1000; ++i) {
        if (i >= end) { break; }
        rewardSeries->append(static_cast<qreal>(end - i), rewardTrain[end - i]);
        valueSeries->append(static_cast<qreal>(end - i), valueTrain[end - i]);
        valueDotSeries->append(static_cast<qreal>(end - i), valueDotTrain[end - i]);
        tdSeries->append(static_cast<qreal>(end - i), tdTrain[end - i]);
    }
    rewardChart->addSeries(rewardSeries);
    rewardChart->addSeries(valueSeries);
    rewardChart->addSeries(valueDotSeries);
    rewardChart->addSeries(tdSeries);
    rewardChart->createDefaultAxes();
    rewardChart->legend()->setVisible(true);
    rewardChart->legend()->setAlignment(Qt::AlignBottom);
    ui->rewardView->update();
}

void NeuvisysGUISimulation::onDisplayAction(const std::vector<std::vector<double>> &actionTrain) {
    auto nbActions = actionSeries.size();
    actionSeries.clear();
    actionChart->removeAllSeries();

    auto end = actionTrain[0].size();
    for (auto i = 0; i < nbActions; ++i) {
        actionSeries.push_back(new QLineSeries());
        actionSeries[i]->setName(QString::fromStdString("Action " + std::to_string(i)));

        for (auto j = 1; j < 1000; ++j) {
            if (j >= end) { break; }
            actionSeries[i]->append(static_cast<qreal>(end - j), actionTrain[i][end - j]);
        }

        actionChart->addSeries(actionSeries[i]);
    }

    actionChart->createDefaultAxes();
    actionChart->legend()->setVisible(true);
    actionChart->legend()->setAlignment(Qt::AlignBottom);
    ui->actionView->update();
}

void NeuvisysGUISimulation::on_slider_precision_event_sliderMoved(int position) {
    precisionEvent = static_cast<size_t>(position);
    ui->lcd_precision_event->display(static_cast<double>(precisionEvent) / 1000);
    emit precisionEventChanged(precisionEvent);
}

void NeuvisysGUISimulation::on_slider_range_potential_sliderMoved(int position) {
    rangePotential = static_cast<size_t>(position);
    ui->lcd_range_potential->display(static_cast<double>(rangePotential) / 1000);
    emit rangePotentialChanged(rangePotential);
}

void NeuvisysGUISimulation::on_slider_precision_potential_sliderMoved(int position) {
    precisionPotential = static_cast<size_t>(position);
    ui->lcd_precision_potential->display(static_cast<double>(precisionPotential) / 1000);
    emit precisionPotentialChanged(precisionPotential);
}

void NeuvisysGUISimulation::on_slider_range_spiketrain_sliderMoved(int position) {
    rangeSpiketrain = static_cast<size_t>(position);
    ui->lcd_range_spiketrain->display(static_cast<double>(rangeSpiketrain) / 1000);
    emit rangeSpikeTrainChanged(rangeSpiketrain);
}

void NeuvisysGUISimulation::on_slider_layer_sliderMoved(int position) {
    m_layer = position;
    m_id = 0;
    m_zcell = 0;
    emit layerChanged(m_layer);
}

void NeuvisysGUISimulation::onConsoleMessage(const std::string &msg) {
    ui->console->insertPlainText(QString::fromStdString(msg));
}

void NeuvisysGUISimulation::onNetworkDestruction() {
    ui->console->insertPlainText(QString("Finished.\n"));
}
