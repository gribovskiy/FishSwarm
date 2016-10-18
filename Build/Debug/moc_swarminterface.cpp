/****************************************************************************
** Meta object code from reading C++ file 'swarminterface.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../swarminterface.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'swarminterface.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_SwarmInterface_t {
    QByteArrayData data[25];
    char stringdata0[555];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SwarmInterface_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SwarmInterface_t qt_meta_stringdata_SwarmInterface = {
    {
QT_MOC_LITERAL(0, 0, 14), // "SwarmInterface"
QT_MOC_LITERAL(1, 15, 30), // "SwarmInterface_StartSimulation"
QT_MOC_LITERAL(2, 46, 0), // ""
QT_MOC_LITERAL(3, 47, 29), // "SwarmInterface_StopSimulation"
QT_MOC_LITERAL(4, 77, 21), // "on_LoadButton_clicked"
QT_MOC_LITERAL(5, 99, 22), // "on_StartButton_clicked"
QT_MOC_LITERAL(6, 122, 22), // "on_PauseButton_clicked"
QT_MOC_LITERAL(7, 145, 27), // "on_FishSpinBox_valueChanged"
QT_MOC_LITERAL(8, 173, 12), // "newFishCount"
QT_MOC_LITERAL(9, 186, 25), // "on_KpSpinBox_valueChanged"
QT_MOC_LITERAL(10, 212, 5), // "newKp"
QT_MOC_LITERAL(11, 218, 25), // "on_KiSpinBox_valueChanged"
QT_MOC_LITERAL(12, 244, 5), // "newKi"
QT_MOC_LITERAL(13, 250, 25), // "on_KdSpinBox_valueChanged"
QT_MOC_LITERAL(14, 276, 5), // "newKd"
QT_MOC_LITERAL(15, 282, 37), // "on_LinearVelocitySpinBox_valu..."
QT_MOC_LITERAL(16, 320, 12), // "newLinearVel"
QT_MOC_LITERAL(17, 333, 31), // "on_OmegaMaxSpinBox_valueChanged"
QT_MOC_LITERAL(18, 365, 11), // "newOmegaMax"
QT_MOC_LITERAL(19, 377, 34), // "on_ArenaHeightSpinBox_valueCh..."
QT_MOC_LITERAL(20, 412, 4), // "arg1"
QT_MOC_LITERAL(21, 417, 34), // "on_ArenaLengthSpinBox_valueCh..."
QT_MOC_LITERAL(22, 452, 34), // "on_RobotHeightSpinBox_valueCh..."
QT_MOC_LITERAL(23, 487, 34), // "on_RobotLengthSpinBox_valueCh..."
QT_MOC_LITERAL(24, 522, 32) // "on_DJikstraDrawPathFish1_clicked"

    },
    "SwarmInterface\0SwarmInterface_StartSimulation\0"
    "\0SwarmInterface_StopSimulation\0"
    "on_LoadButton_clicked\0on_StartButton_clicked\0"
    "on_PauseButton_clicked\0"
    "on_FishSpinBox_valueChanged\0newFishCount\0"
    "on_KpSpinBox_valueChanged\0newKp\0"
    "on_KiSpinBox_valueChanged\0newKi\0"
    "on_KdSpinBox_valueChanged\0newKd\0"
    "on_LinearVelocitySpinBox_valueChanged\0"
    "newLinearVel\0on_OmegaMaxSpinBox_valueChanged\0"
    "newOmegaMax\0on_ArenaHeightSpinBox_valueChanged\0"
    "arg1\0on_ArenaLengthSpinBox_valueChanged\0"
    "on_RobotHeightSpinBox_valueChanged\0"
    "on_RobotLengthSpinBox_valueChanged\0"
    "on_DJikstraDrawPathFish1_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SwarmInterface[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x08 /* Private */,
       3,    0,   95,    2, 0x08 /* Private */,
       4,    0,   96,    2, 0x08 /* Private */,
       5,    0,   97,    2, 0x08 /* Private */,
       6,    0,   98,    2, 0x08 /* Private */,
       7,    1,   99,    2, 0x08 /* Private */,
       9,    1,  102,    2, 0x08 /* Private */,
      11,    1,  105,    2, 0x08 /* Private */,
      13,    1,  108,    2, 0x08 /* Private */,
      15,    1,  111,    2, 0x08 /* Private */,
      17,    1,  114,    2, 0x08 /* Private */,
      19,    1,  117,    2, 0x08 /* Private */,
      21,    1,  120,    2, 0x08 /* Private */,
      22,    1,  123,    2, 0x08 /* Private */,
      23,    1,  126,    2, 0x08 /* Private */,
      24,    0,  129,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, QMetaType::Int,   12,
    QMetaType::Void, QMetaType::Int,   14,
    QMetaType::Void, QMetaType::Int,   16,
    QMetaType::Void, QMetaType::Int,   18,
    QMetaType::Void, QMetaType::Int,   20,
    QMetaType::Void, QMetaType::Int,   20,
    QMetaType::Void, QMetaType::Int,   20,
    QMetaType::Void, QMetaType::Int,   20,
    QMetaType::Void,

       0        // eod
};

void SwarmInterface::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SwarmInterface *_t = static_cast<SwarmInterface *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SwarmInterface_StartSimulation(); break;
        case 1: _t->SwarmInterface_StopSimulation(); break;
        case 2: _t->on_LoadButton_clicked(); break;
        case 3: _t->on_StartButton_clicked(); break;
        case 4: _t->on_PauseButton_clicked(); break;
        case 5: _t->on_FishSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->on_KpSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_KiSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->on_KdSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_LinearVelocitySpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_OmegaMaxSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->on_ArenaHeightSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_ArenaLengthSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->on_RobotHeightSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->on_RobotLengthSpinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->on_DJikstraDrawPathFish1_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject SwarmInterface::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_SwarmInterface.data,
      qt_meta_data_SwarmInterface,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *SwarmInterface::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SwarmInterface::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_SwarmInterface.stringdata0))
        return static_cast<void*>(const_cast< SwarmInterface*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int SwarmInterface::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
