/****************************************************************************
** Meta object code from reading C++ file 'Neuvisysthread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "include/gui/Neuvisysthread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Neuvisysthread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_NeuvisysThread_t {
    QByteArrayData data[71];
    char stringdata0[1059];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_NeuvisysThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_NeuvisysThread_t qt_meta_stringdata_NeuvisysThread = {
    {
QT_MOC_LITERAL(0, 0, 14), // "NeuvisysThread"
QT_MOC_LITERAL(1, 15, 15), // "displayProgress"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 8), // "progress"
QT_MOC_LITERAL(4, 41, 4), // "time"
QT_MOC_LITERAL(5, 46, 17), // "displayStatistics"
QT_MOC_LITERAL(6, 64, 19), // "std::vector<double>"
QT_MOC_LITERAL(7, 84, 14), // "eventRateTrain"
QT_MOC_LITERAL(8, 99, 16), // "networkRateTrain"
QT_MOC_LITERAL(9, 116, 13), // "displayEvents"
QT_MOC_LITERAL(10, 130, 7), // "cv::Mat"
QT_MOC_LITERAL(11, 138, 16), // "leftEventDisplay"
QT_MOC_LITERAL(12, 155, 17), // "rightEventDisplay"
QT_MOC_LITERAL(13, 173, 14), // "displayWeights"
QT_MOC_LITERAL(14, 188, 24), // "std::map<size_t,cv::Mat>"
QT_MOC_LITERAL(15, 213, 13), // "weightDisplay"
QT_MOC_LITERAL(16, 227, 6), // "size_t"
QT_MOC_LITERAL(17, 234, 5), // "layer"
QT_MOC_LITERAL(18, 240, 16), // "displayPotential"
QT_MOC_LITERAL(19, 257, 9), // "spikeRate"
QT_MOC_LITERAL(20, 267, 6), // "vreset"
QT_MOC_LITERAL(21, 274, 9), // "threshold"
QT_MOC_LITERAL(22, 284, 38), // "std::vector<std::pair<double,..."
QT_MOC_LITERAL(23, 323, 14), // "potentialTrain"
QT_MOC_LITERAL(24, 338, 12), // "displaySpike"
QT_MOC_LITERAL(25, 351, 64), // "std::vector<std::reference_wr..."
QT_MOC_LITERAL(26, 416, 10), // "spikeTrain"
QT_MOC_LITERAL(27, 427, 13), // "displayReward"
QT_MOC_LITERAL(28, 441, 11), // "rewardTrain"
QT_MOC_LITERAL(29, 453, 10), // "valueTrain"
QT_MOC_LITERAL(30, 464, 13), // "valueDotTrain"
QT_MOC_LITERAL(31, 478, 7), // "tdTrain"
QT_MOC_LITERAL(32, 486, 13), // "displayAction"
QT_MOC_LITERAL(33, 500, 12), // "action1Train"
QT_MOC_LITERAL(34, 513, 12), // "action2Train"
QT_MOC_LITERAL(35, 526, 20), // "networkConfiguration"
QT_MOC_LITERAL(36, 547, 11), // "std::string"
QT_MOC_LITERAL(37, 559, 11), // "sharingType"
QT_MOC_LITERAL(38, 571, 33), // "std::vector<std::vector<size_..."
QT_MOC_LITERAL(39, 605, 12), // "layerPatches"
QT_MOC_LITERAL(40, 618, 19), // "std::vector<size_t>"
QT_MOC_LITERAL(41, 638, 10), // "layerSizes"
QT_MOC_LITERAL(42, 649, 11), // "neuronSizes"
QT_MOC_LITERAL(43, 661, 15), // "networkCreation"
QT_MOC_LITERAL(44, 677, 9), // "nbCameras"
QT_MOC_LITERAL(45, 687, 10), // "nbSynapses"
QT_MOC_LITERAL(46, 698, 16), // "networkStructure"
QT_MOC_LITERAL(47, 715, 7), // "vfWidth"
QT_MOC_LITERAL(48, 723, 8), // "vfHeight"
QT_MOC_LITERAL(49, 732, 18), // "networkDestruction"
QT_MOC_LITERAL(50, 751, 14), // "consoleMessage"
QT_MOC_LITERAL(51, 766, 3), // "msg"
QT_MOC_LITERAL(52, 770, 15), // "onTabVizChanged"
QT_MOC_LITERAL(53, 786, 5), // "index"
QT_MOC_LITERAL(54, 792, 14), // "onIndexChanged"
QT_MOC_LITERAL(55, 807, 14), // "onZcellChanged"
QT_MOC_LITERAL(56, 822, 5), // "zcell"
QT_MOC_LITERAL(57, 828, 15), // "onCameraChanged"
QT_MOC_LITERAL(58, 844, 6), // "camera"
QT_MOC_LITERAL(59, 851, 16), // "onSynapseChanged"
QT_MOC_LITERAL(60, 868, 7), // "synapse"
QT_MOC_LITERAL(61, 876, 23), // "onPrecisionEventChanged"
QT_MOC_LITERAL(62, 900, 11), // "displayRate"
QT_MOC_LITERAL(63, 912, 23), // "onRangePotentialChanged"
QT_MOC_LITERAL(64, 936, 14), // "rangePotential"
QT_MOC_LITERAL(65, 951, 27), // "onPrecisionPotentialChanged"
QT_MOC_LITERAL(66, 979, 9), // "trackRate"
QT_MOC_LITERAL(67, 989, 24), // "onRangeSpikeTrainChanged"
QT_MOC_LITERAL(68, 1014, 15), // "rangeSpiketrain"
QT_MOC_LITERAL(69, 1030, 14), // "onLayerChanged"
QT_MOC_LITERAL(70, 1045, 13) // "onStopNetwork"

    },
    "NeuvisysThread\0displayProgress\0\0"
    "progress\0time\0displayStatistics\0"
    "std::vector<double>\0eventRateTrain\0"
    "networkRateTrain\0displayEvents\0cv::Mat\0"
    "leftEventDisplay\0rightEventDisplay\0"
    "displayWeights\0std::map<size_t,cv::Mat>\0"
    "weightDisplay\0size_t\0layer\0displayPotential\0"
    "spikeRate\0vreset\0threshold\0"
    "std::vector<std::pair<double,size_t> >\0"
    "potentialTrain\0displaySpike\0"
    "std::vector<std::reference_wrapper<const std::vector<size_t> > >\0"
    "spikeTrain\0displayReward\0rewardTrain\0"
    "valueTrain\0valueDotTrain\0tdTrain\0"
    "displayAction\0action1Train\0action2Train\0"
    "networkConfiguration\0std::string\0"
    "sharingType\0std::vector<std::vector<size_t> >\0"
    "layerPatches\0std::vector<size_t>\0"
    "layerSizes\0neuronSizes\0networkCreation\0"
    "nbCameras\0nbSynapses\0networkStructure\0"
    "vfWidth\0vfHeight\0networkDestruction\0"
    "consoleMessage\0msg\0onTabVizChanged\0"
    "index\0onIndexChanged\0onZcellChanged\0"
    "zcell\0onCameraChanged\0camera\0"
    "onSynapseChanged\0synapse\0"
    "onPrecisionEventChanged\0displayRate\0"
    "onRangePotentialChanged\0rangePotential\0"
    "onPrecisionPotentialChanged\0trackRate\0"
    "onRangeSpikeTrainChanged\0rangeSpiketrain\0"
    "onLayerChanged\0onStopNetwork"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_NeuvisysThread[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      23,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      12,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,  129,    2, 0x06 /* Public */,
       5,    2,  134,    2, 0x06 /* Public */,
       9,    2,  139,    2, 0x06 /* Public */,
      13,    2,  144,    2, 0x06 /* Public */,
      18,    4,  149,    2, 0x06 /* Public */,
      24,    2,  158,    2, 0x06 /* Public */,
      27,    4,  163,    2, 0x06 /* Public */,
      32,    2,  172,    2, 0x06 /* Public */,
      35,    4,  177,    2, 0x06 /* Public */,
      43,    5,  186,    2, 0x06 /* Public */,
      49,    0,  197,    2, 0x06 /* Public */,
      50,    1,  198,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      52,    1,  201,    2, 0x0a /* Public */,
      54,    1,  204,    2, 0x0a /* Public */,
      55,    1,  207,    2, 0x0a /* Public */,
      57,    1,  210,    2, 0x0a /* Public */,
      59,    1,  213,    2, 0x0a /* Public */,
      61,    1,  216,    2, 0x0a /* Public */,
      63,    1,  219,    2, 0x0a /* Public */,
      65,    1,  222,    2, 0x0a /* Public */,
      67,    1,  225,    2, 0x0a /* Public */,
      69,    1,  228,    2, 0x0a /* Public */,
      70,    0,  231,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Double,    3,    4,
    QMetaType::Void, 0x80000000 | 6, 0x80000000 | 6,    7,    8,
    QMetaType::Void, 0x80000000 | 10, 0x80000000 | 10,   11,   12,
    QMetaType::Void, 0x80000000 | 14, 0x80000000 | 16,   15,   17,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, 0x80000000 | 22,   19,   20,   21,   23,
    QMetaType::Void, 0x80000000 | 25, QMetaType::Double,   26,    4,
    QMetaType::Void, 0x80000000 | 6, 0x80000000 | 6, 0x80000000 | 6, 0x80000000 | 6,   28,   29,   30,   31,
    QMetaType::Void, 0x80000000 | 6, 0x80000000 | 6,   33,   34,
    QMetaType::Void, 0x80000000 | 36, 0x80000000 | 38, 0x80000000 | 40, 0x80000000 | 40,   37,   39,   41,   42,
    QMetaType::Void, 0x80000000 | 16, 0x80000000 | 16, 0x80000000 | 40, 0x80000000 | 16, 0x80000000 | 16,   44,   45,   46,   47,   48,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 36,   51,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 16,   53,
    QMetaType::Void, 0x80000000 | 16,   53,
    QMetaType::Void, 0x80000000 | 16,   56,
    QMetaType::Void, 0x80000000 | 16,   58,
    QMetaType::Void, 0x80000000 | 16,   60,
    QMetaType::Void, 0x80000000 | 16,   62,
    QMetaType::Void, 0x80000000 | 16,   64,
    QMetaType::Void, 0x80000000 | 16,   66,
    QMetaType::Void, 0x80000000 | 16,   68,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void,

       0        // eod
};

void NeuvisysThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<NeuvisysThread *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->displayProgress((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->displayStatistics((*reinterpret_cast< const std::vector<double>(*)>(_a[1])),(*reinterpret_cast< const std::vector<double>(*)>(_a[2]))); break;
        case 2: _t->displayEvents((*reinterpret_cast< const cv::Mat(*)>(_a[1])),(*reinterpret_cast< const cv::Mat(*)>(_a[2]))); break;
        case 3: _t->displayWeights((*reinterpret_cast< const std::map<size_t,cv::Mat>(*)>(_a[1])),(*reinterpret_cast< size_t(*)>(_a[2]))); break;
        case 4: _t->displayPotential((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< const std::vector<std::pair<double,size_t> >(*)>(_a[4]))); break;
        case 5: _t->displaySpike((*reinterpret_cast< const std::vector<std::reference_wrapper<const std::vector<size_t> > >(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 6: _t->displayReward((*reinterpret_cast< const std::vector<double>(*)>(_a[1])),(*reinterpret_cast< const std::vector<double>(*)>(_a[2])),(*reinterpret_cast< const std::vector<double>(*)>(_a[3])),(*reinterpret_cast< const std::vector<double>(*)>(_a[4]))); break;
        case 7: _t->displayAction((*reinterpret_cast< const std::vector<double>(*)>(_a[1])),(*reinterpret_cast< const std::vector<double>(*)>(_a[2]))); break;
        case 8: _t->networkConfiguration((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const std::vector<std::vector<size_t> >(*)>(_a[2])),(*reinterpret_cast< const std::vector<size_t>(*)>(_a[3])),(*reinterpret_cast< const std::vector<size_t>(*)>(_a[4]))); break;
        case 9: _t->networkCreation((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< size_t(*)>(_a[2])),(*reinterpret_cast< const std::vector<size_t>(*)>(_a[3])),(*reinterpret_cast< const size_t(*)>(_a[4])),(*reinterpret_cast< const size_t(*)>(_a[5]))); break;
        case 10: _t->networkDestruction(); break;
        case 11: _t->consoleMessage((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 12: _t->onTabVizChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 13: _t->onIndexChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 14: _t->onZcellChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 15: _t->onCameraChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 16: _t->onSynapseChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 17: _t->onPrecisionEventChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 18: _t->onRangePotentialChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 19: _t->onPrecisionPotentialChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 20: _t->onRangeSpikeTrainChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 21: _t->onLayerChanged((*reinterpret_cast< size_t(*)>(_a[1]))); break;
        case 22: _t->onStopNetwork(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (NeuvisysThread::*)(int , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::displayProgress)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)(const std::vector<double> & , const std::vector<double> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::displayStatistics)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)(const cv::Mat & , const cv::Mat & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::displayEvents)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)(const std::map<size_t,cv::Mat> & , size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::displayWeights)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)(double , double , double , const std::vector<std::pair<double,size_t>> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::displayPotential)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)(const std::vector<std::reference_wrapper<const std::vector<size_t>> > & , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::displaySpike)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)(const std::vector<double> & , const std::vector<double> & , const std::vector<double> & , const std::vector<double> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::displayReward)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)(const std::vector<double> & , const std::vector<double> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::displayAction)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)(const std::string & , const std::vector<std::vector<size_t>> & , const std::vector<size_t> & , const std::vector<size_t> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::networkConfiguration)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)(size_t , size_t , const std::vector<size_t> & , const size_t , const size_t );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::networkCreation)) {
                *result = 9;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::networkDestruction)) {
                *result = 10;
                return;
            }
        }
        {
            using _t = void (NeuvisysThread::*)(const std::string & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&NeuvisysThread::consoleMessage)) {
                *result = 11;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject NeuvisysThread::staticMetaObject = { {
    &QThread::staticMetaObject,
    qt_meta_stringdata_NeuvisysThread.data,
    qt_meta_data_NeuvisysThread,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *NeuvisysThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *NeuvisysThread::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_NeuvisysThread.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int NeuvisysThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 23)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 23;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 23)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 23;
    }
    return _id;
}

// SIGNAL 0
void NeuvisysThread::displayProgress(int _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void NeuvisysThread::displayStatistics(const std::vector<double> & _t1, const std::vector<double> & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void NeuvisysThread::displayEvents(const cv::Mat & _t1, const cv::Mat & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void NeuvisysThread::displayWeights(const std::map<size_t,cv::Mat> & _t1, size_t _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void NeuvisysThread::displayPotential(double _t1, double _t2, double _t3, const std::vector<std::pair<double,size_t>> & _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void NeuvisysThread::displaySpike(const std::vector<std::reference_wrapper<const std::vector<size_t>> > & _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void NeuvisysThread::displayReward(const std::vector<double> & _t1, const std::vector<double> & _t2, const std::vector<double> & _t3, const std::vector<double> & _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void NeuvisysThread::displayAction(const std::vector<double> & _t1, const std::vector<double> & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void NeuvisysThread::networkConfiguration(const std::string & _t1, const std::vector<std::vector<size_t>> & _t2, const std::vector<size_t> & _t3, const std::vector<size_t> & _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void NeuvisysThread::networkCreation(size_t _t1, size_t _t2, const std::vector<size_t> & _t3, const size_t _t4, const size_t _t5)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void NeuvisysThread::networkDestruction()
{
    QMetaObject::activate(this, &staticMetaObject, 10, nullptr);
}

// SIGNAL 11
void NeuvisysThread::consoleMessage(const std::string & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
