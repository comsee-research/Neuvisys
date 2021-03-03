#include "neuvisysgui.h"
#include "./ui_neuvisysgui.h"

NeuvisysGUI::NeuvisysGUI(QWidget *parent) : QMainWindow(parent), ui(new Ui::NeuvisysGUI) {
    index = 0;
    index2 = 0;
    layer = 0;
    layer2 = 0;
    camera = 0;
    synapse = 0;
    precisionEvent = 30000;
    precisionPotential = 10000;
    rangePotential = 5000000;
    precisionSpiketrain = 100000;
    rangeSpiketrain = 1000000;

    ui->setupUi(this);
    ui->text_event_file->setText("/home/alphat/Desktop/Events/2_1.npz");
    ui->text_network_directory->setText("/home/alphat/neuvisys-dv/configuration/network");
    openConfigFiles();
    ui->number_runs->setValue(1);
    ui->progressBar->setValue(0);
    ui->spin_layer_selection->setMinimum(1);

    /* Selection parameters */
    ui->slider_precision_event->setMaximum(100000);
    ui->slider_precision_event->setMinimum(5000);
    ui->slider_precision_event->setValue(precisionEvent);

    ui->slider_precision_potential->setMaximum(100000);
    ui->slider_precision_potential->setMinimum(100);
    ui->slider_precision_potential->setValue(precisionPotential);
    ui->slider_range_potential->setMaximum(5000000);
    ui->slider_range_potential->setMinimum(1000);
    ui->slider_range_potential->setValue(rangePotential);

    ui->slider_precision_spiketrain->setMaximum(10000);
    ui->slider_precision_spiketrain->setMinimum(100);
    ui->slider_precision_spiketrain->setValue(precisionSpiketrain);
    ui->slider_range_spiketrain->setMaximum(5000000);
    ui->slider_range_spiketrain->setMinimum(100000);
    ui->slider_range_spiketrain->setValue(rangeSpiketrain);

    ui->lcd_precision_event->display(static_cast<double>(precisionEvent) / 1000);
    ui->lcd_precision_potential->display(static_cast<double>(precisionPotential) / 1000);
    ui->lcd_range_potential->display(static_cast<double>(rangePotential) / 1000);
    ui->lcd_precision_spiktrain->display(static_cast<double>(precisionSpiketrain) / 1000);
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
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<std::map<size_t, cv::Mat>>("std::map<size_t, cv::Mat>");
    qRegisterMetaType<std::vector<std::pair<double,long>>>("std::vector<std::pair<double,long>>");
    qRegisterMetaType<std::map<size_t, std::vector<long>>>("std::map<size_t, std::vector<long>>");

    connect(&neuvisysThread, &NeuvisysThread::displayInformation, this, &NeuvisysGUI::onDisplayInformation);
    connect(&neuvisysThread, &NeuvisysThread::networkConfiguration, this, &NeuvisysGUI::onNetworkConfiguration);
    connect(this, &NeuvisysGUI::guiInformation, &neuvisysThread, &NeuvisysThread::onGuiInformation);
    neuvisysThread.render(ui->text_network_directory->text()+"/configs/network_config.json", ui->text_event_file->text(), ui->number_runs->value());
}

void NeuvisysGUI::onNetworkConfiguration(const int nbCameras, const int nbSynapses, const std::string sharingType, const int width, const int height, const int depth, const int widthPatchSize, const int heightPatchSize) {
    ui->spin_layer_selection->setMaximum(depth);
    ui->spin_camera_selection->setMaximum(nbCameras-1);
    ui->spin_synapse_selection->setMaximum(nbSynapses-1);

    int count = 0;
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            QPushButton *button = new QPushButton(this);
            button->setText(QString::number(count));
            button->setMinimumWidth(5);
            ui->gridSelection->addWidget(button, i, j);
            connect(button, SIGNAL(clicked()), this, SLOT(on_button_selection_clicked()));
            button->show();
            ++count;

            if (sharingType == "none") {
                QLabel *label = new QLabel(this);
                ui->weightLayout->addWidget(label, i, j);
                label->show();

            }
        }
    }
    if (sharingType == "patch") {
        for (int wp = 0; wp < widthPatchSize; ++wp) {
            for (int hp = 0; hp < heightPatchSize; ++hp) {
                for (int i = 0; i < std::sqrt(depth); ++i) {
                    for (int j = 0; j < std::sqrt(depth); ++j) {
                        QLabel *label = new QLabel(this);
                        ui->weightLayout->addWidget(label, wp*depth+i, hp*depth+j);
                        label->show();
                    }
                }
            }
        }

    } else if (sharingType == "full") {
        for (int i = 0; i < std::sqrt(depth); ++i) {
            for (int j = 0; j < std::sqrt(depth); ++j) {
                QLabel *label = new QLabel(this);
                ui->weightLayout->addWidget(label, i, j);
                label->show();
            }
        }
    }
}

void NeuvisysGUI::on_button_selection_clicked() {
    QPushButton *button = (QPushButton*) sender();
    index = ui->spin_layer_selection->value() * button->text().toInt();
    emit guiInformation(index, layer, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain, precisionSpiketrain);
}

void NeuvisysGUI::on_spin_layer_selection_valueChanged(int arg1) {
    layer = arg1;
    emit guiInformation(index, layer, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain, precisionSpiketrain);
}

void NeuvisysGUI::on_spin_camera_selection_valueChanged(int arg1) {
    camera = arg1;
    emit guiInformation(index, layer, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain, precisionSpiketrain);
}

void NeuvisysGUI::on_spin_synapse_selection_valueChanged(int arg1) {
    synapse = arg1;
    emit guiInformation(index, layer, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain, precisionSpiketrain);
}


void NeuvisysGUI::onDisplayInformation(const int progress, const double spike_rate, const double threshold, const cv::Mat leftEventDisplay, cv::Mat rightEventDisplay, std::map<size_t, cv::Mat> weightDisplay, const std::vector<std::pair<double, long>> &potentialTrain, const std::map<size_t, std::vector<long>> &spikeTrain) {
    ui->lcd_spike_rate->display(spike_rate);
    ui->lcd_threshold->display(threshold);
    ui->progressBar->setValue(progress);

    /*** Events ***/
    cv::Mat temp;
    cvtColor(leftEventDisplay, temp, cv::COLOR_BGR2RGB);
    QImage leftEvent((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    leftEvent.bits();
    ui->left_event_image->setPixmap(QPixmap::fromImage(leftEvent).scaled(3*346, 3*260, Qt::KeepAspectRatio));
    cvtColor(rightEventDisplay, temp, cv::COLOR_BGR2RGB);
    QImage rightEvent((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    rightEvent.bits();
    ui->right_event_image->setPixmap(QPixmap::fromImage(rightEvent).scaled(3*346, 3*260, Qt::KeepAspectRatio));

    /*** Weights ***/
    int count = 0;
    for (auto it = weightDisplay.begin(); it != weightDisplay.end(); ++it) {
        cvtColor(it->second, temp, cv::COLOR_BGR2RGB);
        QImage weight((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
        weight.bits();
        auto label = (QLabel*) ui->weightLayout->itemAt(count)->widget();
        label->setPixmap(QPixmap::fromImage(weight).scaled(40, 40, Qt::KeepAspectRatio));
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
        potentialSeries->append(it->second, it->first);
        if (last - it->second > rangePotential) {
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
    for (auto sTrain = spikeTrain.begin(); sTrain != spikeTrain.end(); ++sTrain) {
        if (!sTrain->second.empty() && sTrain->second.back() > max) {
            max = sTrain->second.back();
        }
        for (auto spike = sTrain->second.rbegin(); spike != sTrain->second.rend(); ++spike) {
            spikeSeries->append(*spike, sTrain->first);
            if (max - *spike > rangeSpiketrain) {
                break;
            }
        }
    }
    spikeChart->addSeries(spikeSeries);
    spikeChart->createDefaultAxes();
    ui->spikeView->repaint();
}

void NeuvisysGUI::on_slider_precision_event_sliderMoved(int position) {
    precisionEvent = position;
    ui->lcd_precision_event->display(static_cast<double>(precisionEvent) / 1000);
    emit guiInformation(index, layer, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain, precisionSpiketrain);
}

void NeuvisysGUI::on_slider_range_potential_sliderMoved(int position) {
    rangePotential = position;
    ui->lcd_range_potential->display(static_cast<double>(rangePotential) / 1000);
    emit guiInformation(index, layer, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain, precisionSpiketrain);
}

void NeuvisysGUI::on_slider_precision_potential_sliderMoved(int position) {
    precisionPotential = position;
    ui->lcd_precision_potential->display(static_cast<double>(precisionPotential) / 1000);
    emit guiInformation(index, layer, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain, precisionSpiketrain);
}

void NeuvisysGUI::on_slider_range_spiketrain_sliderMoved(int position) {
    rangeSpiketrain = position;
    ui->lcd_range_spiketrain->display(static_cast<double>(rangeSpiketrain) / 1000);
    emit guiInformation(index, layer, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain, precisionSpiketrain);
}

void NeuvisysGUI::on_slider_precision_spiketrain_sliderMoved(int position) {
    precisionSpiketrain = position;
    ui->lcd_precision_spiktrain->display(static_cast<double>(precisionSpiketrain) / 1000);
    emit guiInformation(index, layer, camera, synapse, precisionEvent, rangePotential, precisionPotential, rangeSpiketrain, precisionSpiketrain);
}
