/****************************************************************************
** Meta object code from reading C++ file 'Neuvisysgui.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Neuvisysgui.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Neuvisysgui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_NeuvisysGUI_t {
    QByteArrayData data[97];
    char stringdata0[1882];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_NeuvisysGUI_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_NeuvisysGUI_t qt_meta_stringdata_NeuvisysGUI = {
    {
QT_MOC_LITERAL(0, 0, 11), // "NeuvisysGUI"
QT_MOC_LITERAL(1, 12, 13), // "tabVizChanged"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 6), // "size_t"
QT_MOC_LITERAL(4, 34, 5), // "index"
QT_MOC_LITERAL(5, 40, 12), // "indexChanged"
QT_MOC_LITERAL(6, 53, 12), // "zcellChanged"
QT_MOC_LITERAL(7, 66, 5), // "zcell"
QT_MOC_LITERAL(8, 72, 13), // "cameraChanged"
QT_MOC_LITERAL(9, 86, 6), // "camera"
QT_MOC_LITERAL(10, 93, 14), // "synapseChanged"
QT_MOC_LITERAL(11, 108, 7), // "synapse"
QT_MOC_LITERAL(12, 116, 21), // "precisionEventChanged"
QT_MOC_LITERAL(13, 138, 14), // "precisionEvent"
QT_MOC_LITERAL(14, 153, 21), // "rangePotentialChanged"
QT_MOC_LITERAL(15, 175, 14), // "rangePotential"
QT_MOC_LITERAL(16, 190, 25), // "precisionPotentialChanged"
QT_MOC_LITERAL(17, 216, 18), // "precisionPotential"
QT_MOC_LITERAL(18, 235, 22), // "rangeSpikeTrainChanged"
QT_MOC_LITERAL(19, 258, 15), // "rangeSpiketrain"
QT_MOC_LITERAL(20, 274, 12), // "layerChanged"
QT_MOC_LITERAL(21, 287, 5), // "layer"
QT_MOC_LITERAL(22, 293, 11), // "stopNetwork"
QT_MOC_LITERAL(23, 305, 13), // "createNetwork"
QT_MOC_LITERAL(24, 319, 11), // "std::string"
QT_MOC_LITERAL(25, 331, 8), // "fileName"
QT_MOC_LITERAL(26, 340, 17), // "onDisplayProgress"
QT_MOC_LITERAL(27, 358, 8), // "progress"
QT_MOC_LITERAL(28, 367, 4), // "time"
QT_MOC_LITERAL(29, 372, 19), // "onDisplayStatistics"
QT_MOC_LITERAL(30, 392, 10), // "event_rate"
QT_MOC_LITERAL(31, 403, 12), // "on_off_ratio"
QT_MOC_LITERAL(32, 416, 10), // "spike_rate"
QT_MOC_LITERAL(33, 427, 9), // "threshold"
QT_MOC_LITERAL(34, 437, 4), // "bias"
QT_MOC_LITERAL(35, 442, 15), // "onDisplayEvents"
QT_MOC_LITERAL(36, 458, 7), // "cv::Mat"
QT_MOC_LITERAL(37, 466, 16), // "leftEventDisplay"
QT_MOC_LITERAL(38, 483, 17), // "rightEventDisplay"
QT_MOC_LITERAL(39, 501, 16), // "onDisplayWeights"
QT_MOC_LITERAL(40, 518, 24), // "std::map<size_t,cv::Mat>"
QT_MOC_LITERAL(41, 543, 13), // "weightDisplay"
QT_MOC_LITERAL(42, 557, 8), // "layerViz"
QT_MOC_LITERAL(43, 566, 18), // "onDisplayPotential"
QT_MOC_LITERAL(44, 585, 6), // "vreset"
QT_MOC_LITERAL(45, 592, 38), // "std::vector<std::pair<double,..."
QT_MOC_LITERAL(46, 631, 14), // "potentialTrain"
QT_MOC_LITERAL(47, 646, 14), // "onDisplaySpike"
QT_MOC_LITERAL(48, 661, 64), // "std::vector<std::reference_wr..."
QT_MOC_LITERAL(49, 726, 11), // "spikeTrains"
QT_MOC_LITERAL(50, 738, 15), // "onDisplayReward"
QT_MOC_LITERAL(51, 754, 19), // "std::vector<double>"
QT_MOC_LITERAL(52, 774, 11), // "rewardTrain"
QT_MOC_LITERAL(53, 786, 10), // "valueTrain"
QT_MOC_LITERAL(54, 797, 13), // "valueDotTrain"
QT_MOC_LITERAL(55, 811, 7), // "tdTrain"
QT_MOC_LITERAL(56, 819, 15), // "onDisplayAction"
QT_MOC_LITERAL(57, 835, 12), // "action1Train"
QT_MOC_LITERAL(58, 848, 12), // "action2Train"
QT_MOC_LITERAL(59, 861, 22), // "onNetworkConfiguration"
QT_MOC_LITERAL(60, 884, 11), // "sharingType"
QT_MOC_LITERAL(61, 896, 33), // "std::vector<std::vector<size_..."
QT_MOC_LITERAL(62, 930, 12), // "layerPatches"
QT_MOC_LITERAL(63, 943, 19), // "std::vector<size_t>"
QT_MOC_LITERAL(64, 963, 10), // "layerSizes"
QT_MOC_LITERAL(65, 974, 11), // "neuronSizes"
QT_MOC_LITERAL(66, 986, 17), // "onNetworkCreation"
QT_MOC_LITERAL(67, 1004, 9), // "nbCameras"
QT_MOC_LITERAL(68, 1014, 10), // "nbSynapses"
QT_MOC_LITERAL(69, 1025, 16), // "networkStructure"
QT_MOC_LITERAL(70, 1042, 20), // "onNetworkDestruction"
QT_MOC_LITERAL(71, 1063, 16), // "onConsoleMessage"
QT_MOC_LITERAL(72, 1080, 3), // "msg"
QT_MOC_LITERAL(73, 1084, 28), // "on_button_event_file_clicked"
QT_MOC_LITERAL(74, 1113, 35), // "on_button_network_directory_c..."
QT_MOC_LITERAL(75, 1149, 32), // "on_button_create_network_clicked"
QT_MOC_LITERAL(76, 1182, 32), // "on_button_launch_network_clicked"
QT_MOC_LITERAL(77, 1215, 34), // "on_text_network_config_textCh..."
QT_MOC_LITERAL(78, 1250, 29), // "on_text_rl_config_textChanged"
QT_MOC_LITERAL(79, 1280, 38), // "on_text_simple_cell_config_te..."
QT_MOC_LITERAL(80, 1319, 39), // "on_text_complex_cell_config_t..."
QT_MOC_LITERAL(81, 1359, 38), // "on_text_critic_cell_config_te..."
QT_MOC_LITERAL(82, 1398, 37), // "on_text_actor_cell_config_tex..."
QT_MOC_LITERAL(83, 1436, 37), // "on_text_network_directory_tex..."
QT_MOC_LITERAL(84, 1474, 27), // "on_button_selection_clicked"
QT_MOC_LITERAL(85, 1502, 35), // "on_tab_visualization_currentC..."
QT_MOC_LITERAL(86, 1538, 36), // "on_spin_zcell_selection_value..."
QT_MOC_LITERAL(87, 1575, 4), // "arg1"
QT_MOC_LITERAL(88, 1580, 37), // "on_spin_camera_selection_valu..."
QT_MOC_LITERAL(89, 1618, 38), // "on_spin_synapse_selection_val..."
QT_MOC_LITERAL(90, 1657, 37), // "on_slider_precision_event_sli..."
QT_MOC_LITERAL(91, 1695, 8), // "position"
QT_MOC_LITERAL(92, 1704, 37), // "on_slider_range_potential_sli..."
QT_MOC_LITERAL(93, 1742, 41), // "on_slider_precision_potential..."
QT_MOC_LITERAL(94, 1784, 38), // "on_slider_range_spiketrain_sl..."
QT_MOC_LITERAL(95, 1823, 27), // "on_slider_layer_sliderMoved"
QT_MOC_LITERAL(96, 1851, 30) // "on_button_stop_network_clicked"

    },
    "NeuvisysGUI\0tabVizChanged\0\0size_t\0"
    "index\0indexChanged\0zcellChanged\0zcell\0"
    "cameraChanged\0camera\0synapseChanged\0"
    "synapse\0precisionEventChanged\0"
    "precisionEvent\0rangePotentialChanged\0"
    "rangePotential\0precisionPotentialChanged\0"
    "precisionPotential\0rangeSpikeTrainChanged\0"
    "rangeSpiketrain\0layerChanged\0layer\0"
    "stopNetwork\0createNetwork\0std::string\0"
    "fileName\0onDisplayProgress\0progress\0"
    "time\0onDisplayStatistics\0event_rate\0"
    "on_off_ratio\0spike_rate\0threshold\0"
    "bias\0onDisplayEvents\0cv::Mat\0"
    "leftEventDisplay\0rightEventDisplay\0"
    "onDisplayWeights\0std::map<size_t,cv::Mat>\0"
    "weightDisplay\0layerViz\0onDisplayPotential\0"
    "vreset\0std::vector<std::pair<double,size_t> >\0"
    "potentialTrain\0onDisplaySpike\0"
    "std::vector<std::reference_wrapper<const std::vector<size_t> > >\0"
    "spikeTrains\0onDisplayReward\0"
    "std::vector<double>\0rewardTrain\0"
    "valueTrain\0valueDotTrain\0tdTrain\0"
    "onDisplayAction\0action1Train\0action2Train\0"
    "onNetworkConfiguration\0sharingType\0"
    "std::vector<std::vector<size_t> >\0"
    "layerPatches\0std::vector<size_t>\0"
    "layerSizes\0neuronSizes\0onNetworkCreation\0"
    "nbCameras\0nbSynapses\0networkStructure\0"
    "onNetworkDestruction\0onConsoleMessage\0"
    "msg\0on_button_event_file_clicked\0"
    "on_button_network_directory_clicked\0"
    "on_button_create_network_clicked\0"
    "on_button_launch_network_clicked\0"
    "on_text_network_config_textChanged\0"
    "on_text_rl_config_textChanged\0"
    "on_text_simple_cell_config_textChanged\0"
    "on_text_complex_cell_config_textChanged\0"
    "on_text_critic_cell_config_textChanged\0"
    "on_text_actor_cell_config_textChanged\0"
    "on_text_network_directory_textChanged\0"
    "on_button_selection_clicked\0"
    "on_tab_visualization_currentChanged\0"
    "on_spin_zcell_selection_valueChanged\0"
    "arg1\0on_spin_camera_selection_valueChanged\0"
    "on_spin_synapse_selection_valueChanged\0"
    "on_slider_precision_event_sliderMoved\0"
    "position\0on_slider_range_potential_sliderMoved\0"
    "on_slider_precision_potential_sliderMoved\0"
    "on_slider_range_spiketrain_sliderMoved\0"
    "on_slider_layer_sliderMoved\0"
    "on_button_stop_network_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_NeuvisysGUI[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      46,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      12,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  244,    2, 0x06 /* Public */,
       5,    1,  247,    2, 0x06 /* Public */,
       6,    1,  250,    2, 0x06 /* Public */,
       8,    1,  253,    2, 0x06 /* Public */,
      10,    1,  256,    2, 0x06 /* Public */,
      12,    1,  259,    2, 0x06 /* Public */,
      14,    1,  262,    2, 0x06 /* Public */,
      16,    1,  265,    2, 0x06 /* Public */,
      18,    1,  268,    2, 0x06 /* Public */,
      20,    1,  271,    2, 0x06 /* Public */,
      22,    0,  274,    2, 0x06 /* Public */,
      23,    1,  275,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      26,    2,  278,    2, 0x0a /* Public */,
      29,    5,  283,    2, 0x0a /* Public */,
      35,    2,  294,    2, 0x0a /* Public */,
      39,    2,  299,    2, 0x0a /* Public */,
      43,    3,  304,    2, 0x0a /* Public */,
      47,    2,  311,    2, 0x0a /* Public */,
      50,    4,  316,    2, 0x0a /* Public */,
      56,    2,  325,    2, 0x0a /* Public */,
      59,    4,  330,    2, 0x0a /* Public */,
      66,    3,  339,    2, 0x0a /* Public */,
      70,    0,  346,    2, 0x0a /* Public */,
      71,    1,  347,    2, 0x0a /* Public */,
      73,    0,  350,    2, 0x08 /* Private */,
      74,    0,  351,    2, 0x08 /* Private */,
      75,    0,  352,    2, 0x08 /* Private */,
      76,    0,  353,    2, 0x08 /* Private */,
      77,    0,  354,    2, 0x08 /* Private */,
      78,    0,  355,    2, 0x08 /* Private */,
      79,    0,  356,    2, 0x08 /* Private */,
      80,    0,  357,    2, 0x08 /* Private */,
      81,    0,  358,    2, 0x08 /* Private */,
      82,    0,  359,    2, 0x08 /* Private */,
      83,    0,  360,    2, 0x08 /* Private */,
      84,    1,  361,    2, 0x08 /* Private */,
      85,    1,  364,    2, 0x08 /* Private */,
      86,    1,  367,    2, 0x08 /* Private */,
      88,    1,  370,    2, 0x08 /* Private */,
      89,    1,  373,    2, 0x08 /* Private */,
      90,    1,  376,    2, 0x08 /* Private */,
      92,    1,  379,    2, 0x08 /* Private */,
      93,    1,  382,    2, 0x08 /* Private */,
      94,    1,  385,    2, 0x08 /* Private */,
      95,    1,  388,    2, 0x08 /* Private */,
      96,    0,  391,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    7,
    QMetaType::Void, 0x80000000 | 3,    9,
    QMetaType::Void, 0x80000000 | 3,   11,
    QMetaType::Void, 0x80000000 | 3,   13,
    QMetaType::Void, 0x80000000 | 3,   15,
    QMetaType::Void, 0x80000000 | 3,   17,
    QMetaType::Void, 0x80000000 | 3,   19,
    QMetaType::Void, 0x80000000 | 3,   21,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 24,   25,

 // slots: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Double,   27,   28,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   30,   31,   32,   33,   34,
    QMetaType::Void, 0x80000000 | 36, 0x80000000 | 36,   37,   38,
    QMetaType::Void, 0x80000000 | 40, 0x80000000 | 3,   41,   42,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, 0x80000000 | 45,   44,   33,   46,
    QMetaType::Void, 0x80000000 | 48, QMetaType::Double,   49,   28,
    QMetaType::Void, 0x80000000 | 51, 0x80000000 | 51, 0x80000000 | 51, 0x80000000 | 51,   52,   53,   54,   55,
    QMetaType::Void, 0x80000000 | 51, 0x80000000 | 51,   57,   58,
    QMetaType::Void, 0x80000000 | 24, 0x80000000 | 61, 0x80000000 | 63, 0x80000000 | 63,   60,   62,   64,   65,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 3, 0x80000000 | 63,   67,   68,   69,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 24,   72,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void, QMetaType::Int,    4,
    QMetaType::Void, QMetaType::Int,   87,
    QMetaType::Void, QMetaType::Int,   87,
    QMetaType::Void, QMetaType::Int,   87,
    QMetaType::Void, QMetaType::Int,   91,
    QMetaType::Void, QMetaType::Int,   91,
    QMetaType::Void, QMetaType::Int,   91,
    QMetaType::Void, QMetaType::Int,   91,
    QMetaType::Void, QMetaType::Int,   91,
    QMetaType::Void,

       0        // eod
};

void NeuvisysGUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<NeuvisysGUI *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->tabVizChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 1: _t->indexChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 2: _t->zcellChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 3: _t->cameraChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 4: _t->synapseChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 5: _t->precisionEventChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 6: _t->rangePotentialChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 7: _t->precisionPotentialChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 8: _t->rangeSpikeTrainChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 9: _t->layerChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 10: _t->stopNetwork(); break;
        case 11: _t->createNetwork((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 12: _t->onDisplayProgress((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 13: _t->onDisplayStatistics((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5]))); break;
        case 14: _t->onDisplayEvents((*reinterpret_cast< const cv::Mat(*)>(_a[1])),(*reinterpret_cast< const cv::Mat(*)>(_a[2]))); break;
        case 15: _t->onDisplayWeights((*reinterpret_cast< const std::map<size_t,cv::Mat>(*)>(_a[1])),(*reinterpret_cast< size_t(*)>(_a[2]))); break;
        case 16: _t->onDisplayPotential((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< const std::vector<std::pair<double,size_t> >(*)>(_a[3]))); break;
        case 17: _t->onDisplaySpike((*reinterpret_cast< const std::vector<std::reference_wrapper<const std::vector<size_t> > >(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 18: _t->onDisplayReward((*reinterpret_cast< const std::vector<double>(*)>(_a[1])),(*reinterpret_cast< const std::vector<double>(*)>(_a[2])),(*reinterpret_cast< const std::vector<double>(*)>(_a[3])),(*reinterpret_cast< const std::vector<double>(*)>(_a[4]))); break;
        case 19: _t->onDisplayAction((*reinterpret_cast< const std::vector<double>(*)>(_a[1])),(*reinterpret_cast< const std::vector<double>(*)>(_a[2]))); break;
        case 20: _t->onNetworkConfiguration((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const std::vector<std::vector<size_t> >(*)>(_a[2])),(*reinterpret_cast< const std::vector<size_t>(*)>(_a[3])),(*reinterpret_cast< const std::vector<size_t>(*)>(_a[4]))); break;
        case 21: _t->onNetworkCreation((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< size_t(*)>(_a[2])),(*reinterpret_cast< const std::vector<size_t>(*)>(_a[3]))); break;
        case 22: _t->onNetworkDestruction(); break;
        case 23: _t->onConsoleMessage((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 24: _t->on_button_event_file_clicked(); break;
        case 25: _t->on_button_network_directory_clicked(); break;
        case 26: _t->on_button_create_network_clicked(); break;
        case 27: _t->on_button_launch_network_clicked(); break;
        case 28: _t->on_text_network_config_textChanged(); break;
        case 29: _t->on_text_rl_config_textChanged(); break;
        case 30: _t->on_text_simple_cell_config_textChanged(); break;
        case 31: _t->on_text_complex_cell_config_textChanged(); break;
        case 32: _t->on_text_critic_cell_config_textChanged(); break;
        case 33: _t->on_text_actor_cell_config_textChanged(); break;
        case 34: _t->on_text_network_directory_textChanged(); break;
        case 35: _t->on_button_selection_clicked((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 36: _t->on_tab_visualization_currentChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 37: _t->on_spin_zcell_selection_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 38: _t->on_spin_camera_selection_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 39: _t->on_spin_synapse_selection_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 40: _t->on_slider_precision_event_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 41: _t->on_slider_range_potential_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 42: _t->on_slider_precision_potential_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 43: _t->on_slider_range_spiketrain_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 44: _t->on_slider_layer_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 45: _t->on_button_stop_network_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (NeuvisysGUI::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::tabVizChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::indexChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::zcellChanged)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::cameraChanged)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::synapseChanged)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::precisionEventChanged)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::rangePotentialChanged)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::precisionPotentialChanged)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::rangeSpikeTrainChanged)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)(size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::layerChanged)) {
                *result = 9;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::stopNetwork)) {
                *result = 10;
                return;
            }
        }
        {
            using _t = void (NeuvisysGUI::*)(std::string );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysGUI::createNetwork)) {
                *result = 11;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject NeuvisysGUI::staticMetaObject = { {
    &QMainWindow::staticMetaObject,
    qt_meta_stringdata_NeuvisysGUI.data,
    qt_meta_data_NeuvisysGUI,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *NeuvisysGUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *NeuvisysGUI::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_NeuvisysGUI.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int NeuvisysGUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 46)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 46;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 46)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 46;
    }
    return _id;
}

// SIGNAL 0
void NeuvisysGUI::tabVizChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void NeuvisysGUI::indexChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void NeuvisysGUI::zcellChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void NeuvisysGUI::cameraChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void NeuvisysGUI::synapseChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void NeuvisysGUI::precisionEventChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void NeuvisysGUI::rangePotentialChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void NeuvisysGUI::precisionPotentialChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void NeuvisysGUI::rangeSpikeTrainChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void NeuvisysGUI::layerChanged(size_t _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void NeuvisysGUI::stopNetwork()
{
    QMetaObject::activate(this, &staticMetaObject, 10, nullptr);
}

// SIGNAL 11
void NeuvisysGUI::createNetwork(std::string _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE