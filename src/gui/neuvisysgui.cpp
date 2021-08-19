#include "neuvisysgui.h"
#include "./ui_neuvisysgui.h"

NeuvisysGUI::NeuvisysGUI(int argc, char** argv, QWidget *parent) : QMainWindow(parent), ui(new Ui::NeuvisysGUI), neuvisysThread(argc, argv) {
    precisionEvent = 30000;
    precisionPotential = 10000;
    rangePotential = 1000000;
    rangeSpiketrain = 1000000;

    ui->setupUi(this);
    ui->text_event_file->setText("/home/thomas/Desktop/shapes.npz");
    ui->text_network_directory->setText("/home/thomas/neuvisys-dv/configuration/network");
    openConfigFiles();
    ui->number_runs->setValue(1);
    ui->progressBar->setValue(0);
    ui->spin_depth_selection->setMinimum(0);

    ui->left_event_video->setScene(new QGraphicsScene(this));
    ui->left_event_video->scene()->addItem(&leftEvents);
    ui->right_event_video->setScene(new QGraphicsScene(this));
    ui->right_event_video->scene()->addItem(&rightEvents);

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
    ui->slider_range_spiketrain->setMaximum(5000000);
    ui->slider_range_spiketrain->setMinimum(100000);
    ui->slider_range_spiketrain->setValue(static_cast<int>(rangeSpiketrain));

    ui->lcd_precision_event->display(static_cast<double>(precisionEvent) / 1000);
    ui->lcd_precision_potential->display(static_cast<double>(precisionPotential) / 1000);
    ui->lcd_range_potential->display(static_cast<double>(rangePotential) / 1000);
    ui->lcd_range_spiketrain->display(static_cast<double>(rangeSpiketrain) / 1000);

    /* Qt charts */
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
    rewardChart = new QChart();
    rewardChart->legend()->hide();
    rewardChart->addSeries(rewardSeries);
    rewardChart->createDefaultAxes();
    rewardChart->setTitle("Reward plot");
    ui->rewardView->setChart(rewardChart);
    ui->rewardView->setRenderHint(QPainter::Antialiasing);
}

NeuvisysGUI::~NeuvisysGUI() {
    delete ui;
    free(potentialSeries);
    free(potentialChart);
    free(spikeSeries);
    free(spikeChart);
    free(rewardSeries);
    free(rewardChart);

    if (ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }
}

void NeuvisysGUI::on_button_event_file_clicked() {
    QString fileName = QFileDialog::getOpenFileName(this, "Open the file");
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    ui->text_event_file->setText(fileName);
    file.close();
}

void NeuvisysGUI::on_button_network_directory_clicked() {
    QString dir = QFileDialog::getExistingDirectory(this, "Open Directory", "/home", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    ui->text_network_directory->setText(dir);
    openConfigFiles();
}

void NeuvisysGUI::openConfigFiles() {
    QString dir = ui->text_network_directory->text();
    QString confFile = dir + "/configs/network_config.json";
    QFile file(confFile);
    if (!file.open(QIODevice::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    QTextStream networkText(&file);
    ui->text_network_config->setText(networkText.readAll());
    file.close();

    confFile = dir + "/configs/simple_cell_config.json";
    file.setFileName(confFile);
    if (!file.open(QIODevice::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    QTextStream simpleText(&file);
    ui->text_simple_cell_config->setText(simpleText.readAll());
    file.close();

    confFile = dir + "/configs/complex_cell_config.json";
    file.setFileName(confFile);
    if (!file.open(QIODevice::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    QTextStream complexText(&file);
    ui->text_complex_cell_config->setText(complexText.readAll());
    file.close();

    confFile = dir + "/configs/motor_cell_config.json";
    file.setFileName(confFile);
    if (!file.open(QIODevice::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    QTextStream motorText(&file);
    ui->text_motor_cell_config->setText(complexText.readAll());
    file.close();
}

void NeuvisysGUI::on_text_network_config_textChanged() {
    QString dir = ui->text_network_directory->text();
    QString confFile = dir + "/configs/network_config.json";
    QFile file(confFile);
    if (!file.open(QIODevice::WriteOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    QTextStream out(&file);
    QString text = ui->text_network_config->toPlainText();
    out << text;
    file.close();
}

void NeuvisysGUI::on_text_simple_cell_config_textChanged() {
    QString dir = ui->text_network_directory->text();
    QString confFile = dir + "/configs/simple_cell_config.json";
    QFile file(confFile);
    if (!file.open(QIODevice::WriteOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    QTextStream out(&file);
    QString text = ui->text_simple_cell_config->toPlainText();
    out << text;
    file.close();
}

void NeuvisysGUI::on_text_complex_cell_config_textChanged() {
    QString dir = ui->text_network_directory->text();
    QString confFile = dir + "/configs/complex_cell_config.json";
    QFile file(confFile);
    if (!file.open(QIODevice::WriteOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    QTextStream out(&file);
    QString text = ui->text_complex_cell_config->toPlainText();
    out << text;
    file.close();
}

void NeuvisysGUI::on_text_motor_cell_config_textChanged() {
    QString dir = ui->text_network_directory->text();
    QString confFile = dir + "/configs/motor_cell_config.json";
    QFile file(confFile);
    if (!file.open(QIODevice::WriteOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    QTextStream out(&file);
    QString text = ui->text_motor_cell_config->toPlainText();
    out << text;
    file.close();
}

void NeuvisysGUI::on_button_launch_network_clicked() {
    qRegisterMetaType<size_t>("size_t");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<std::map<size_t, cv::Mat>>("std::map<size_t, cv::Mat>");
    qRegisterMetaType<std::vector<std::pair<double, long>>>("std::vector<std::pair<double,long>>");
    qRegisterMetaType<std::map<size_t, std::vector<long>>>("std::map<size_t, std::vector<long>>");
    qRegisterMetaType<std::vector<double>>("std::vector<double>");
    qRegisterMetaType<std::vector<bool>>("std::vector<bool>");
    qRegisterMetaType<std::vector<size_t>>("std::vector<size_t>");
    qRegisterMetaType<std::vector<std::vector<size_t>>>("std::vector<std::vector<size_t>>");

    connect(&neuvisysThread, &NeuvisysThread::displayProgress, this, &NeuvisysGUI::onDisplayProgress);
    connect(&neuvisysThread, &NeuvisysThread::displayEvents, this, &NeuvisysGUI::onDisplayEvents);
    connect(&neuvisysThread, &NeuvisysThread::displayPotential, this, &NeuvisysGUI::onDisplayPotential);
    connect(&neuvisysThread, &NeuvisysThread::displaySpike, this, &NeuvisysGUI::onDisplaySpike);
    connect(&neuvisysThread, &NeuvisysThread::displayWeights, this, &NeuvisysGUI::onDisplayWeights);
    connect(&neuvisysThread, &NeuvisysThread::displayReward, this, &NeuvisysGUI::onDisplayReward);
    connect(&neuvisysThread, &NeuvisysThread::displayAction, this, &NeuvisysGUI::onDisplayAction);
    connect(&neuvisysThread, &NeuvisysThread::networkConfiguration, this, &NeuvisysGUI::onNetworkConfiguration);
    connect(&neuvisysThread, &NeuvisysThread::networkCreation, this, &NeuvisysGUI::onNetworkCreation);
    connect(&neuvisysThread, &NeuvisysThread::networkDestruction, this, &NeuvisysGUI::onFinished);
    neuvisysThread.init();

    connect(this, &NeuvisysGUI::indexChanged, &neuvisysThread, &NeuvisysThread::onIndexChanged);
    connect(this, &NeuvisysGUI::zcellChanged, &neuvisysThread, &NeuvisysThread::onZcellChanged);
    connect(this, &NeuvisysGUI::depthChanged, &neuvisysThread, &NeuvisysThread::onDepthChanged);
    connect(this, &NeuvisysGUI::cameraChanged, &neuvisysThread, &NeuvisysThread::onCameraChanged);
    connect(this, &NeuvisysGUI::synapseChanged, &neuvisysThread, &NeuvisysThread::onSynapseChanged);
    connect(this, &NeuvisysGUI::precisionEventChanged, &neuvisysThread, &NeuvisysThread::onPrecisionEventChanged);
    connect(this, &NeuvisysGUI::rangePotentialChanged, &neuvisysThread, &NeuvisysThread::onRangePotentialChanged);
    connect(this, &NeuvisysGUI::precisionPotentialChanged, &neuvisysThread, &NeuvisysThread::onPrecisionPotentialChanged);
    connect(this, &NeuvisysGUI::rangeSpikeTrainChanged, &neuvisysThread, &NeuvisysThread::onRangeSpikeTrainChanged);
    connect(this, &NeuvisysGUI::layerChanged, &neuvisysThread, &NeuvisysThread::onLayerChanged);
    connect(this, &NeuvisysGUI::stopNetwork, &neuvisysThread, &NeuvisysThread::onStopNetwork);

    neuvisysThread.render(ui->text_network_directory->text() + "/configs/network_config.json", ui->text_event_file->text(),
                          static_cast<size_t>(ui->number_runs->value()), ui->realtime->isChecked());
    ui->console->insertPlainText(QString("Starting network...\n"));
}

void NeuvisysGUI::on_button_stop_network_clicked() {
    emit stopNetwork();
    ui->console->insertPlainText(QString("Saving network...\n"));
}

void NeuvisysGUI::onNetworkConfiguration(const std::string &sharingType, const std::vector<std::vector<size_t>> &layerPatches, const std::vector<size_t> &layerSizes, const
std::vector<size_t> &neuronSizes) {
    ui->spin_depth_selection->setMaximum(static_cast<int>(neuronSizes[2]-1));
    ui->spin_zcell_selection->setMaximum(static_cast<int>(layerSizes[2]-1));

    while (QLayoutItem* item = ui->gridSelection->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    while (QLayoutItem* item = ui->weightLayout->takeAt(0)) {
        delete item->widget();
        delete item;
    }

    int count = 0;
    for (size_t i = 0; i < layerPatches[0].size() * layerSizes[0]; ++i) {
        for (size_t j = 0; j < layerPatches[1].size() * layerSizes[1]; ++j) {
            auto *button = new QPushButton(this);
            button->setText(QString::number(count));
            button->setMinimumWidth(5);
            ui->gridSelection->addWidget(button, static_cast<int>(i), static_cast<int>(j));
            connect(button, SIGNAL(clicked()), this, SLOT(on_button_selection_clicked()));
            button->show();
            ++count;

            if (sharingType == "none") {
                auto *label = new QLabel(this);
                ui->weightLayout->addWidget(label, static_cast<int>(i), static_cast<int>(j));
                label->show();
            }
        }
    }
    if (sharingType == "patch") {
        for (size_t wp = 0; wp < layerPatches[0].size(); ++wp) {
            for (size_t hp = 0; hp < layerPatches[1].size(); ++hp) {
                for (size_t i = 0; i < static_cast<size_t>(std::sqrt(layerSizes[2])); ++i) {
                    for (size_t j = 0; j < static_cast<size_t>(std::sqrt(layerSizes[2])); ++j) {
                        auto *label = new QLabel(this);
                        ui->weightLayout->addWidget(label, static_cast<int>(hp * layerSizes[2] + i), static_cast<int>(wp * layerSizes[2] + j));
                        label->show();
                    }
                }
            }
        }
    } else if (sharingType == "full") {
        for (int i = 0; i < std::sqrt(layerSizes[2]); ++i) {
            for (int j = 0; j < std::sqrt(layerSizes[2]); ++j) {
                auto *label = new QLabel(this);
                ui->weightLayout->addWidget(label, i, j);
                label->show();
            }
        }
    }
}

void NeuvisysGUI::onNetworkCreation(const size_t nbCameras, const size_t nbSynapses, const std::vector<size_t> &networkStructure) {
    ui->spin_camera_selection->setMaximum(static_cast<int>(nbCameras - 1));
    ui->spin_synapse_selection->setMaximum(static_cast<int>(nbSynapses - 1));
    ui->slider_layer->setMaximum(static_cast<int>(networkStructure.size()-1));

    QString message = QString("Network structure: ");
    for (auto structure : networkStructure) {
        message.append(QString::number(structure));
        message.append(QString(" / "));
    }
    message.append(QString("\n"));
    ui->console->insertPlainText(message);

    std::vector<QString> labels = { QString("Left"), QString("Right") };
    for (size_t i = 0; i < networkStructure[networkStructure.size()-1]; ++i) {
        auto *label = new QLabel(this);
        label->setText(QString(labels[i]));

        ui->actionGrid->addWidget(label, 0, static_cast<int>(i));
        label->show();
    }
}

void NeuvisysGUI::on_button_selection_clicked() {
    auto *button = dynamic_cast<QPushButton *>(sender());
    m_id = static_cast<size_t>(ui->spin_depth_selection->value()) * button->text().toULong();
}

void NeuvisysGUI::on_spin_zcell_selection_valueChanged(int arg1) {
    m_zcell = static_cast<size_t>(arg1);
    emit zcellChanged(m_zcell);
}

void NeuvisysGUI::on_spin_depth_selection_valueChanged(int arg1) {
    m_depth = static_cast<size_t>(arg1);
    emit depthChanged(m_depth);
}

void NeuvisysGUI::on_spin_camera_selection_valueChanged(int arg1) {
    m_camera = static_cast<size_t>(arg1);
    emit cameraChanged(m_camera);
}

void NeuvisysGUI::on_spin_synapse_selection_valueChanged(int arg1) {
    m_synapse = static_cast<size_t>(arg1);
    emit synapseChanged(m_synapse);
}

void NeuvisysGUI::onDisplayProgress(int progress, double spike_rate, double threshold, double bias) {
    ui->lcd_spike_rate->display(spike_rate);
    ui->lcd_threshold->display(threshold);
    ui->lcd_bias->display(bias);
    ui->progressBar->setValue(progress);
}

void NeuvisysGUI::onDisplayEvents(const cv::Mat &leftEventDisplay, const cv::Mat& rightEventDisplay) {
    QImage leftEFrame(static_cast<const uchar *>(leftEventDisplay.data), leftEventDisplay.cols, leftEventDisplay.rows,
                      static_cast<int>(leftEventDisplay.step), QImage::Format_RGB888);
    leftEvents.setPixmap(QPixmap::fromImage(leftEFrame.rgbSwapped()).scaled(static_cast<int>(1.5 * 346), static_cast<int>(1.5 * 260)));
    ui->left_event_video->fitInView(&leftEvents, Qt::KeepAspectRatio);

    QImage rightEFrame(static_cast<const uchar *>(rightEventDisplay.data), rightEventDisplay.cols, rightEventDisplay.rows,
                       static_cast<int>(rightEventDisplay.step), QImage::Format_RGB888);
    rightEvents.setPixmap(QPixmap::fromImage(rightEFrame.rgbSwapped()).scaled(static_cast<int>(1.5 * 346), static_cast<int>(1.5 * 260)));
    ui->right_event_video->fitInView(&rightEvents, Qt::KeepAspectRatio);
}

void NeuvisysGUI::onDisplayWeights(const std::map<size_t, cv::Mat> &weightDisplay, const size_t layerViz) {
    int count = 0;
    for (auto &weight : weightDisplay) {
        QImage weightImage(static_cast<const uchar *>(weight.second.data), weight.second.cols, weight.second.rows,
                           static_cast<int>(weight.second.step), QImage::Format_RGB888);
        if (count < ui->weightLayout->count()) {
            auto label = dynamic_cast<QLabel *>(ui->weightLayout->itemAt(count)->widget());
            if (m_layer == 0) {
                label->setPixmap(QPixmap::fromImage(weightImage.rgbSwapped()).scaled(40, 40, Qt::KeepAspectRatio));
            } else {
                label->setPixmap(QPixmap::fromImage(weightImage.rgbSwapped()).scaled(500, 500, Qt::KeepAspectRatio));
            }
        }
        ++count;
    }
}

void NeuvisysGUI::onDisplayPotential(double vreset, double threshold, const std::vector<std::pair<double, long>> &potentialTrain) {
    potentialChart->removeSeries(potentialSeries);
    potentialSeries = new QLineSeries();
    long last = 0;
    if (!potentialTrain.empty()) {
        last = potentialTrain.back().second;
    }
    for (auto it = potentialTrain.rbegin(); it != potentialTrain.rend(); ++it) {
        potentialSeries->append(static_cast<qreal>(it->second), it->first);
        if (last - it->second > static_cast<long>(rangePotential)) {
            break;
        }
    }
    potentialChart->addSeries(potentialSeries);
    potentialChart->createDefaultAxes();
    potentialChart->axes(Qt::Vertical, potentialSeries)[0]->setRange(vreset, threshold);
    ui->potentialView->repaint();
}

void NeuvisysGUI::onDisplaySpike(const std::map<size_t, std::vector<long>> &spikeTrain) {
    spikeChart->removeSeries(spikeSeries);
    spikeSeries = new QScatterSeries();
    spikeSeries->setMarkerShape(QScatterSeries::MarkerShapeRectangle);
    spikeSeries->setMarkerSize(8);
    long max = 0;
    for (const auto &sTrain : spikeTrain) {
        if (!sTrain.second.empty() && sTrain.second.back() > max) {
            max = sTrain.second.back();
        }
    }
    for (const auto &sTrain : spikeTrain) {
        for (auto spike = sTrain.second.rbegin(); spike != sTrain.second.rend(); ++spike) {
            if (max - *spike <= static_cast<long>(rangeSpiketrain)) {
                spikeSeries->append(static_cast<qreal>(*spike), static_cast<qreal>(sTrain.first));
            } else {
                break;
            }
        }
    }
    spikeChart->addSeries(spikeSeries);
    spikeChart->createDefaultAxes();
    ui->spikeView->repaint();
}

void NeuvisysGUI::onDisplayReward(const std::vector<double> &rewardTrain) {
    rewardChart->removeSeries(rewardSeries);
    rewardSeries = new QLineSeries();
    long count = 10000;
    for (auto it = rewardTrain.rbegin(); it != rewardTrain.rend(); ++it) {
        if (count < 0) {
            break;
        }
        rewardSeries->append(static_cast<qreal>(count), *it);
        --count;
    }
    rewardChart->addSeries(rewardSeries);
    rewardChart->createDefaultAxes();
    ui->rewardView->repaint();
}

void NeuvisysGUI::onDisplayAction(const std::vector<bool> &motorActivation) {
    int count = 0;
    for (auto action : motorActivation) {
        if (action) {
            auto label = ui->actionGrid->itemAt(count)->widget();
            label->setStyleSheet("QLabel { background-color : red; color : blue; }");
        } else {
            auto label = ui->actionGrid->itemAt(count)->widget();
            label->setStyleSheet("QLabel { background-color : blue; color : red; }");
        }
        ++count;
    }
}

void NeuvisysGUI::on_slider_precision_event_sliderMoved(int position) {
    precisionEvent = static_cast<size_t>(position);
    ui->lcd_precision_event->display(static_cast<double>(precisionEvent) / 1000);
    emit precisionEventChanged(precisionEvent);
}

void NeuvisysGUI::on_slider_range_potential_sliderMoved(int position) {
    rangePotential = static_cast<size_t>(position);
    ui->lcd_range_potential->display(static_cast<double>(rangePotential) / 1000);
    emit rangePotentialChanged(rangePotential);
}

void NeuvisysGUI::on_slider_precision_potential_sliderMoved(int position) {
    precisionPotential = static_cast<size_t>(position);
    ui->lcd_precision_potential->display(static_cast<double>(precisionPotential) / 1000);
    emit precisionPotentialChanged(precisionPotential);
}

void NeuvisysGUI::on_slider_range_spiketrain_sliderMoved(int position) {
    rangeSpiketrain = static_cast<size_t>(position);
    ui->lcd_range_spiketrain->display(static_cast<double>(rangeSpiketrain) / 1000);
    emit rangeSpikeTrainChanged(rangeSpiketrain);
}

void NeuvisysGUI::on_slider_layer_sliderMoved(int position) {
    m_layer = position;
    emit layerChanged(m_layer);
}

void NeuvisysGUI::onFinished() {
    ui->console->insertPlainText(QString("Finished."));
}
