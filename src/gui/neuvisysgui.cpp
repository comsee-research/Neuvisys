#include "neuvisysgui.h"
#include "./ui_neuvisysgui.h"

NeuvisysGUI::NeuvisysGUI(int argc, char** argv, QWidget *parent) : QMainWindow(parent), ui(new Ui::NeuvisysGUI), neuvisysThread(argc, argv) {
    id = 0;
    layer = 0;
    camera = 0;
    synapse = 0;
    cellType = 0;
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
    ui->spin_layer_selection->setMinimum(1);

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
}

NeuvisysGUI::~NeuvisysGUI() {
    delete ui;
    free(potentialSeries);
    free(potentialChart);
    free(spikeSeries);
    free(spikeChart);

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

    connect(&neuvisysThread, &NeuvisysThread::displayInformation, this, &NeuvisysGUI::onDisplayInformation);
    connect(&neuvisysThread, &NeuvisysThread::networkConfiguration, this, &NeuvisysGUI::onNetworkConfiguration);
    neuvisysThread.init();

    connect(this, &NeuvisysGUI::indexChanged, &neuvisysThread, &NeuvisysThread::onIndexChanged);
    connect(this, &NeuvisysGUI::layerChanged, &neuvisysThread, &NeuvisysThread::onLayerChanged);
    connect(this, &NeuvisysGUI::cameraChanged, &neuvisysThread, &NeuvisysThread::onCameraChanged);
    connect(this, &NeuvisysGUI::synapseChanged, &neuvisysThread, &NeuvisysThread::onSynapseChanged);
    connect(this, &NeuvisysGUI::precisionEventChanged, &neuvisysThread, &NeuvisysThread::onPrecisionEventChanged);
    connect(this, &NeuvisysGUI::rangePotentialChanged, &neuvisysThread, &NeuvisysThread::onRangePotentialChanged);
    connect(this, &NeuvisysGUI::precisionPotentialChanged, &neuvisysThread, &NeuvisysThread::onPrecisionPotentialChanged);
    connect(this, &NeuvisysGUI::rangeSpikeTrainChanged, &neuvisysThread, &NeuvisysThread::onRangeSpikeTrainChanged);
    connect(this, &NeuvisysGUI::cellTypeChanged, &neuvisysThread, &NeuvisysThread::onCellTypeChanged);
    connect(this, &NeuvisysGUI::stopNetwork, &neuvisysThread, &NeuvisysThread::onStopNetwork);

    neuvisysThread.render(ui->text_network_directory->text() + "/configs/network_config.json", ui->text_event_file->text(),
                          static_cast<size_t>(ui->number_runs->value()), ui->realtime->isChecked());
}

void NeuvisysGUI::on_button_stop_network_clicked() {
    emit stopNetwork();
}

void NeuvisysGUI::onNetworkConfiguration(const size_t nbCameras, const size_t nbSynapses, const std::string &sharingType, const size_t width,
                                         const size_t height, const size_t depth, const size_t widthPatchSize, const size_t heightPatchSize) {
    ui->spin_layer_selection->setMaximum(static_cast<int>(depth));
    ui->spin_camera_selection->setMaximum(static_cast<int>(nbCameras - 1));
    ui->spin_synapse_selection->setMaximum(static_cast<int>(nbSynapses - 1));

    int count = 0;
    for (size_t i = 0; i < width; ++i) {
        for (size_t j = 0; j < height; ++j) {
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
        for (size_t wp = 0; wp < widthPatchSize; ++wp) {
            for (size_t hp = 0; hp < heightPatchSize; ++hp) {
                for (size_t i = 0; i < static_cast<size_t>(std::sqrt(depth)); ++i) {
                    for (size_t j = 0; j < static_cast<size_t>(std::sqrt(depth)); ++j) {
                        auto *label = new QLabel(this);
                        ui->weightLayout->addWidget(label, static_cast<int>(hp * depth + i), static_cast<int>(wp * depth + j));
                        label->show();
                    }
                }
            }
        }
    } else if (sharingType == "full") {
        for (int i = 0; i < std::sqrt(depth); ++i) {
            for (int j = 0; j < std::sqrt(depth); ++j) {
                auto *label = new QLabel(this);
                ui->weightLayout->addWidget(label, i, j);
                label->show();
            }
        }
    }
}

void NeuvisysGUI::on_button_selection_clicked() {
    auto *button = dynamic_cast<QPushButton *>(sender());
    id = static_cast<size_t>(ui->spin_layer_selection->value()) * button->text().toULong();
}

void NeuvisysGUI::on_spin_layer_selection_valueChanged(int arg1) {
    layer = static_cast<size_t>(arg1);
    emit layerChanged(layer);
}

void NeuvisysGUI::on_spin_camera_selection_valueChanged(int arg1) {
    camera = static_cast<size_t>(arg1);
    emit cameraChanged(camera);
}

void NeuvisysGUI::on_spin_synapse_selection_valueChanged(int arg1) {
    synapse = static_cast<size_t>(arg1);
    emit synapseChanged(synapse);
}


void NeuvisysGUI::onDisplayInformation(const int progress, const double spike_rate, const double threshold, const double vreset,
                                       const cv::Mat &leftEventDisplay, const cv::Mat &rightEventDisplay,
                                       const std::map<size_t, cv::Mat> &weightDisplay, const std::vector<std::pair<double, long>> &potentialTrain,
                                       const std::map<size_t, std::vector<long>> &spikeTrain) {
    ui->lcd_spike_rate->display(spike_rate);
    ui->lcd_threshold->display(threshold);
    ui->progressBar->setValue(progress);

    /*** Events ***/
    QImage leftEFrame(static_cast<const uchar *>(leftEventDisplay.data), leftEventDisplay.cols, leftEventDisplay.rows,
                      static_cast<int>(leftEventDisplay.step), QImage::Format_RGB888);
    leftEvents.setPixmap(QPixmap::fromImage(leftEFrame.rgbSwapped()).scaled(static_cast<int>(1.5 * 346), static_cast<int>(1.5 * 260)));
    ui->left_event_video->fitInView(&leftEvents, Qt::KeepAspectRatio);

    QImage rightEFrame(static_cast<const uchar *>(rightEventDisplay.data), rightEventDisplay.cols, rightEventDisplay.rows,
                       static_cast<int>(rightEventDisplay.step), QImage::Format_RGB888);
    rightEvents.setPixmap(QPixmap::fromImage(rightEFrame.rgbSwapped()).scaled(static_cast<int>(1.5 * 346), static_cast<int>(1.5 * 260)));
    ui->right_event_video->fitInView(&rightEvents, Qt::KeepAspectRatio);

    /*** Weights ***/
    int count = 0;
    for (auto &weight : weightDisplay) {
        QImage weightImage(static_cast<const uchar *>(weight.second.data), weight.second.cols, weight.second.rows,
                           static_cast<int>(weight.second.step), QImage::Format_RGB888);
        auto label = dynamic_cast<QLabel *>(ui->weightLayout->itemAt(count)->widget());
        label->setPixmap(QPixmap::fromImage(weightImage.rgbSwapped()).scaled(40, 40, Qt::KeepAspectRatio));
        ++count;
    }

    /*** Potential Plot ***/
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

    /*** Spike Plot ***/
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

void NeuvisysGUI::on_radio_button_simple_cell_clicked() {
    cellType = 0;
    emit cellTypeChanged(cellType);
}

void NeuvisysGUI::on_radio_button_complex_cell_clicked() {
    cellType = 1;
    emit cellTypeChanged(cellType);
}

void NeuvisysGUI::on_radio_button_motor_cell_clicked() {
    cellType = 2;
    emit cellTypeChanged(cellType);
}
