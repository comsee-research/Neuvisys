#include "neuvisysgui.h"
#include "./ui_neuvisysgui.h"

NeuvisysGUI::NeuvisysGUI(QWidget *parent) : QMainWindow(parent), ui(new Ui::NeuvisysGUI) {
    idSimple = 0;
    idComplex = 0;
    layerSimple = 0;
    layerComplex = 0;
    camera = 0;
    synapse = 0;
    precisionEvent = 30000;
    precisionPotential = 10000;
    rangePotential = 1000000;
    rangeSpiketrain = 1000000;

    ui->setupUi(this);
    ui->text_event_file->setText("/home/thomas/Bureau/2_3.npz");
    ui->text_network_directory->setText("/home/thomas/neuvisys-dv/configuration/network");
    openConfigFiles();
    ui->number_runs->setValue(1);
    ui->progressBar->setValue(0);
    ui->spin_layer_selection->setMinimum(1);

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

void NeuvisysGUI::on_button_launch_network_clicked() {
    qRegisterMetaType<size_t>("size_t");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<std::map<size_t, cv::Mat>>("std::map<size_t, cv::Mat>");
    qRegisterMetaType<std::vector<std::pair<double,long>>>("std::vector<std::pair<double,long>>");
    qRegisterMetaType<std::map<size_t, std::vector<long>>>("std::map<size_t, std::vector<long>>");

    connect(&neuvisysThread, &NeuvisysThread::displayInformation, this, &NeuvisysGUI::onDisplayInformation);
    connect(&neuvisysThread, &NeuvisysThread::networkConfiguration, this, &NeuvisysGUI::onNetworkConfiguration);
    connect(this, &NeuvisysGUI::guiInformation, &neuvisysThread, &NeuvisysThread::onGuiInformation);
    neuvisysThread.render(ui->text_network_directory->text()+"/configs/network_config.json", ui->text_event_file->text(),static_cast<size_t>(ui->number_runs->value()));
}

void NeuvisysGUI::onNetworkConfiguration(const size_t nbCameras, const size_t nbSynapses, const std::string &sharingType, const size_t width, const size_t height, const size_t depth, const size_t widthPatchSize, const size_t heightPatchSize) {
    ui->spin_layer_selection->setMaximum(static_cast<int>(depth));
    ui->spin_camera_selection->setMaximum(static_cast<int>(nbCameras-1));
    ui->spin_synapse_selection->setMaximum(static_cast<int>(nbSynapses-1));

    int count = 0;
    for (size_t i = 0; i < width; ++i) {
        for (size_t j = 0; j < height; ++j) {
            auto *button = new QPushButton(this);
            button->setText(QString::number(count));
            button->setMinimumWidth(5);
            ui->gridSelection->addWidget(button, i, j);
            connect(button, SIGNAL(clicked()), this, SLOT(on_button_selection_clicked()));
            button->show();
            ++count;

            if (sharingType == "none") {
                auto *label = new QLabel(this);
                ui->weightLayout->addWidget(label, i, j);
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
                        ui->weightLayout->addWidget(label, wp*depth+i, hp*depth+j);
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
    auto *button = dynamic_cast<QPushButton*>(sender());
    idSimple = static_cast<size_t>(ui->spin_layer_selection->value()) * button->text().toULong();
    emit guiInformation(idSimple, layerSimple, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain);
}

void NeuvisysGUI::on_spin_layer_selection_valueChanged(int arg1) {
    layerSimple = static_cast<size_t>(arg1);
    emit guiInformation(idSimple, layerSimple, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain);
}

void NeuvisysGUI::on_spin_camera_selection_valueChanged(int arg1) {
    camera = static_cast<size_t>(arg1);
    emit guiInformation(idSimple, layerSimple, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain);
}

void NeuvisysGUI::on_spin_synapse_selection_valueChanged(int arg1) {
    synapse = static_cast<size_t>(arg1);
    emit guiInformation(idSimple, layerSimple, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain);
}


void NeuvisysGUI::onDisplayInformation(const int progress, const double spike_rate, const double threshold, const cv::Mat &leftEventDisplay, const cv::Mat &rightEventDisplay, const std::map<size_t, cv::Mat> &weightDisplay, const std::vector<std::pair<double, long>> &potentialTrain, const std::map<size_t, std::vector<long>> &spikeTrain) {
    ui->lcd_spike_rate->display(spike_rate);
    ui->lcd_threshold->display(threshold);
    ui->progressBar->setValue(progress);

    /*** Events ***/
    cv::Mat temp;
    cvtColor(leftEventDisplay, temp, cv::COLOR_BGR2RGB);
    QImage leftEvent(static_cast<const uchar *>(temp.data), temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888);
    leftEvent.bits();
    ui->left_event_image->setPixmap(QPixmap::fromImage(leftEvent).scaled(3*346, 3*260, Qt::KeepAspectRatio));
    cvtColor(rightEventDisplay, temp, cv::COLOR_BGR2RGB);
    QImage rightEvent(static_cast<const uchar *>(temp.data), temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888);
    rightEvent.bits();
    ui->right_event_image->setPixmap(QPixmap::fromImage(rightEvent).scaled(3*346, 3*260, Qt::KeepAspectRatio));

    /*** Weights ***/
    int count = 0;
    for (auto &weight : weightDisplay) {
        cvtColor(weight.second, temp, cv::COLOR_BGR2RGB);
        QImage weightImage(static_cast<const uchar *>(temp.data), temp.cols, temp.rows, static_cast<int>(temp.step), QImage::Format_RGB888);
        weightImage.bits();
        auto label = dynamic_cast<QLabel*>(ui->weightLayout->itemAt(count)->widget());
        label->setPixmap(QPixmap::fromImage(weightImage).scaled(40, 40, Qt::KeepAspectRatio));
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
    emit guiInformation(idSimple, layerSimple, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain);
}

void NeuvisysGUI::on_slider_range_potential_sliderMoved(int position) {
    rangePotential = static_cast<size_t>(position);
    ui->lcd_range_potential->display(static_cast<double>(rangePotential) / 1000);
    emit guiInformation(idSimple, layerSimple, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain);
}

void NeuvisysGUI::on_slider_precision_potential_sliderMoved(int position) {
    precisionPotential = static_cast<size_t>(position);
    ui->lcd_precision_potential->display(static_cast<double>(precisionPotential) / 1000);
    emit guiInformation(idSimple, layerSimple, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain);
}

void NeuvisysGUI::on_slider_range_spiketrain_sliderMoved(int position) {
    rangeSpiketrain = static_cast<size_t>(position);
    ui->lcd_range_spiketrain->display(static_cast<double>(rangeSpiketrain) / 1000);
    emit guiInformation(idSimple, layerSimple, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain);
}
