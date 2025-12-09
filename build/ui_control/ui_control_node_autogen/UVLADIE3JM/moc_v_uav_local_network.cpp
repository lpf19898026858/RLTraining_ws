/****************************************************************************
** Meta object code from reading C++ file 'v_uav_local_network.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/ui_control/src/v_uav_local_network.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'v_uav_local_network.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_v_uav_local_network_t {
    QByteArrayData data[16];
    char stringdata0[458];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_v_uav_local_network_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_v_uav_local_network_t qt_meta_stringdata_v_uav_local_network = {
    {
QT_MOC_LITERAL(0, 0, 19), // "v_uav_local_network"
QT_MOC_LITERAL(1, 20, 19), // "newFeedbackReceived"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 4), // "text"
QT_MOC_LITERAL(4, 46, 20), // "newReasoningReceived"
QT_MOC_LITERAL(5, 67, 39), // "on_v_uav_0_mission_start_butt..."
QT_MOC_LITERAL(6, 107, 37), // "on_v_uav_0_camera_open_button..."
QT_MOC_LITERAL(7, 145, 38), // "on_v_uav_0_camera_close_butto..."
QT_MOC_LITERAL(8, 184, 38), // "on_v_uav_0_send_command_butto..."
QT_MOC_LITERAL(9, 223, 38), // "on_v_uav_0_stop_command_butto..."
QT_MOC_LITERAL(10, 262, 45), // "on_v_uav_0_rereasoning_comman..."
QT_MOC_LITERAL(11, 308, 37), // "on_v_uav_0_run_command_button..."
QT_MOC_LITERAL(12, 346, 21), // "updateFeedbackDisplay"
QT_MOC_LITERAL(13, 368, 22), // "updateReasoningDisplay"
QT_MOC_LITERAL(14, 391, 33), // "on_clear_reasoning_button_cli..."
QT_MOC_LITERAL(15, 425, 32) // "on_clear_feedback_button_clicked"

    },
    "v_uav_local_network\0newFeedbackReceived\0"
    "\0text\0newReasoningReceived\0"
    "on_v_uav_0_mission_start_button_clicked\0"
    "on_v_uav_0_camera_open_button_clicked\0"
    "on_v_uav_0_camera_close_button_clicked\0"
    "on_v_uav_0_send_command_button_clicked\0"
    "on_v_uav_0_stop_command_button_clicked\0"
    "on_v_uav_0_rereasoning_command_button_clicked\0"
    "on_v_uav_0_run_command_button_clicked\0"
    "updateFeedbackDisplay\0updateReasoningDisplay\0"
    "on_clear_reasoning_button_clicked\0"
    "on_clear_feedback_button_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_v_uav_local_network[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   79,    2, 0x06 /* Public */,
       4,    1,   82,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   85,    2, 0x0a /* Public */,
       6,    0,   86,    2, 0x0a /* Public */,
       7,    0,   87,    2, 0x0a /* Public */,
       8,    0,   88,    2, 0x0a /* Public */,
       9,    0,   89,    2, 0x0a /* Public */,
      10,    0,   90,    2, 0x0a /* Public */,
      11,    0,   91,    2, 0x0a /* Public */,
      12,    1,   92,    2, 0x0a /* Public */,
      13,    1,   95,    2, 0x0a /* Public */,
      14,    0,   98,    2, 0x0a /* Public */,
      15,    0,   99,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void v_uav_local_network::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<v_uav_local_network *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newFeedbackReceived((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->newReasoningReceived((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->on_v_uav_0_mission_start_button_clicked(); break;
        case 3: _t->on_v_uav_0_camera_open_button_clicked(); break;
        case 4: _t->on_v_uav_0_camera_close_button_clicked(); break;
        case 5: _t->on_v_uav_0_send_command_button_clicked(); break;
        case 6: _t->on_v_uav_0_stop_command_button_clicked(); break;
        case 7: _t->on_v_uav_0_rereasoning_command_button_clicked(); break;
        case 8: _t->on_v_uav_0_run_command_button_clicked(); break;
        case 9: _t->updateFeedbackDisplay((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 10: _t->updateReasoningDisplay((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 11: _t->on_clear_reasoning_button_clicked(); break;
        case 12: _t->on_clear_feedback_button_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (v_uav_local_network::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&v_uav_local_network::newFeedbackReceived)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (v_uav_local_network::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&v_uav_local_network::newReasoningReceived)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject v_uav_local_network::staticMetaObject = { {
    &QMainWindow::staticMetaObject,
    qt_meta_stringdata_v_uav_local_network.data,
    qt_meta_data_v_uav_local_network,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *v_uav_local_network::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *v_uav_local_network::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_v_uav_local_network.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int v_uav_local_network::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void v_uav_local_network::newFeedbackReceived(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void v_uav_local_network::newReasoningReceived(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
