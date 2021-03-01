#include "neuvisysgui.h"
#include "./ui_neuvisysgui.h"

NeuvisysGUI::NeuvisysGUI(QWidget *parent) : QMainWindow(parent), ui(new Ui::NeuvisysGUI) {
    index = 0;
    index2 = 0;
    layer = 0;
    layer2 = 0;
    camera = 0;
    synapse = 0;
    trackingrate = 1000;
    framerate = 30000;

    ui->setupUi(this);
    ui->text_event_file_1->setText("/home/alphat/Desktop/shape_hovering.npz");
    ui->text_network_directory->setText("/home/alphat/neuvisys-dv/configuration/network");
    openConfigFiles();
    ui->number_runs->setValue(1);
    ui->progressBar->setValue(0);
    ui->spin_layer_selection->setMinimum(1);
    ui->slider_frame_rate->setMaximum(100000);
    ui->slider_frame_rate->setMinimum(5000);
    ui->slider_frame_rate->setValue(framerate);
    ui->slider_tracking_rate->setMaximum(10000);
    ui->slider_tracking_rate->setMinimum(100);
    ui->slider_tracking_rate->setValue(trackingrate);

    ui->lcd_frame_rate->display(static_cast<int>(1000000 / framerate));
    ui->lcd_tracking_rate->display(static_cast<double>(trackingrate) / 1000);

    potentialSeries = new QLineSeries();
    potentialChart = new QChart();
    potentialChart->legend()->hide();
    potentialChart->addSeries(potentialSeries);
    potentialChart->createDefaultAxes();
    potentialChart->setTitle("Potential train of actual neuron");
    ui->potentialView->setChart(potentialChart);
    ui->potentialView->setRenderHint(QPainter::Antialiasing);

    spikeSeries = new QScatterSeries();
    spikeChart = new QChart();
    spikeChart->legend()->hide();
    spikeChart->addSeries(spikeSeries);
    spikeChart->createDefaultAxes();
    spikeChart->setTitle("Potential train of actual neuron");
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

void NeuvisysGUI::on_button_event_file_1_clicked() {
    QString fileName = QFileDialog::getOpenFileName(this, "Open the file");
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    ui->text_event_file_1->setText(fileName);
    file.close();
}

void NeuvisysGUI::on_button_event_file_2_clicked() {
    QString fileName = QFileDialog::getOpenFileName(this, "Open the file");
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QFile::Text)) {
        QMessageBox::warning(this, "Warning", "Cannot open file: " + file.errorString());
        return;
    }
    ui->text_event_file_2->setText(fileName);
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
    neuvisysThread.render(ui->text_network_directory->text()+"/configs/network_config.json", ui->text_event_file_1->text(), ui->text_event_file_2->text(), ui->number_runs->value());
}

void NeuvisysGUI::onNetworkConfiguration(const int nbCameras, const int nbSynapses, const std::string sharingType, const int width, const int height, const int depth) {
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

            if (sharingType == "none" || sharingType == "patch") {
                QLabel *label = new QLabel(this);
                ui->weightLayout->addWidget(label, i, j);
                label->show();

            }
        }
    }
    if (sharingType == "full") {
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
    emit guiInformation(index, layer, camera, synapse, framerate, trackingrate);
}

void NeuvisysGUI::on_spin_layer_selection_valueChanged(int arg1) {
    layer = arg1;
    emit guiInformation(index, layer, camera, synapse, framerate, trackingrate);
}

void NeuvisysGUI::on_spin_camera_selection_valueChanged(int arg1) {
    camera = arg1;
    emit guiInformation(index, layer, camera, synapse, framerate, trackingrate);
}

void NeuvisysGUI::on_spin_synapse_selection_valueChanged(int arg1) {
    synapse = arg1;
    emit guiInformation(index, layer, camera, synapse, framerate, trackingrate);
}

void NeuvisysGUI::on_slider_frame_rate_sliderMoved(int position) {
    framerate = position;
    ui->lcd_frame_rate->display(static_cast<int>(1000000 / framerate));
    emit guiInformation(index, layer, camera, synapse, framerate, trackingrate);
}

void NeuvisysGUI::on_slider_tracking_rate_sliderMoved(int position) {
    trackingrate = position;
    ui->lcd_tracking_rate->display(static_cast<double>(trackingrate) / 1000);
    emit guiInformation(index, layer, camera, synapse, framerate, trackingrate);
}


void NeuvisysGUI::onDisplayInformation(const int progress, const double spike_rate, const cv::Mat eventDisplay, std::map<size_t, cv::Mat> weightDisplay, const std::vector<std::pair<double, long>> &potentialTrain, const std::map<size_t, std::vector<long>> &spikeTrain) {
    ui->text_read_spike_rate->setText(QString::number(spike_rate));
    ui->progressBar->setValue(progress);

    /*** Events ***/
    cv::Mat temp;
    cvtColor(eventDisplay, temp, cv::COLOR_BGR2RGB);
    QImage event((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    event.bits();
    ui->event_image->setPixmap(QPixmap::fromImage(event).scaled(3*346, 3*260, Qt::KeepAspectRatio));

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
    count = 0;
    for (auto it = potentialTrain.rbegin(); it != potentialTrain.rend(); ++it) {
        potentialSeries->append(it->second, it->first);
        if (++count > 5000) {
            break;
        }
    }
    potentialChart->addSeries(potentialSeries);
    potentialChart->createDefaultAxes();
    ui->potentialView->repaint();

    /*** Spike Plot ***/
    spikeChart->removeSeries(spikeSeries);
    spikeSeries = new QScatterSeries();
    long max = 0;
    for (auto sTrain = spikeTrain.begin(); sTrain != spikeTrain.end(); ++sTrain) {
        if (!sTrain->second.empty() && sTrain->second.back() > max) {
            max = sTrain->second.back();
        }
        for (auto spike = sTrain->second.rbegin(); spike != sTrain->second.rend(); ++spike) {
            if (max - *spike < 5000000) {
                spikeSeries->append(*spike, sTrain->first);
            } else {
                break;
            }
        }
    }
    spikeChart->addSeries(spikeSeries);
    spikeChart->createDefaultAxes();
    ui->spikeView->repaint();
}
