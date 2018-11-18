/****************************************************************************
** Meta object code from reading C++ file 'ProcessWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "ProcessWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ProcessWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ProcessWidget_t {
    QByteArrayData data[16];
    char stringdata0[289];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ProcessWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ProcessWidget_t qt_meta_stringdata_ProcessWidget = {
    {
QT_MOC_LITERAL(0, 0, 13), // "ProcessWidget"
QT_MOC_LITERAL(1, 14, 18), // "closeButtonClicked"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 11), // "nameChanged"
QT_MOC_LITERAL(4, 46, 4), // "name"
QT_MOC_LITERAL(5, 51, 19), // "on_pauseBtn_clicked"
QT_MOC_LITERAL(6, 71, 19), // "on_startBtn_clicked"
QT_MOC_LITERAL(7, 91, 19), // "on_colorBtn_clicked"
QT_MOC_LITERAL(8, 111, 19), // "on_closeBtn_clicked"
QT_MOC_LITERAL(9, 131, 23), // "on_exportLogBtn_clicked"
QT_MOC_LITERAL(10, 155, 26), // "on_keepLogCheckBtn_toggled"
QT_MOC_LITERAL(11, 182, 7), // "checked"
QT_MOC_LITERAL(12, 190, 22), // "on_resetLogBtn_clicked"
QT_MOC_LITERAL(13, 213, 23), // "on_drawCheckBtn_toggled"
QT_MOC_LITERAL(14, 237, 23), // "on_syncCheckBox_toggled"
QT_MOC_LITERAL(15, 261, 27) // "on_nameEdit_editingFinished"

    },
    "ProcessWidget\0closeButtonClicked\0\0"
    "nameChanged\0name\0on_pauseBtn_clicked\0"
    "on_startBtn_clicked\0on_colorBtn_clicked\0"
    "on_closeBtn_clicked\0on_exportLogBtn_clicked\0"
    "on_keepLogCheckBtn_toggled\0checked\0"
    "on_resetLogBtn_clicked\0on_drawCheckBtn_toggled\0"
    "on_syncCheckBox_toggled\0"
    "on_nameEdit_editingFinished"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ProcessWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x06 /* Public */,
       3,    1,   75,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   78,    2, 0x08 /* Private */,
       6,    0,   79,    2, 0x08 /* Private */,
       7,    0,   80,    2, 0x08 /* Private */,
       8,    0,   81,    2, 0x08 /* Private */,
       9,    0,   82,    2, 0x08 /* Private */,
      10,    1,   83,    2, 0x08 /* Private */,
      12,    0,   86,    2, 0x08 /* Private */,
      13,    1,   87,    2, 0x08 /* Private */,
      14,    1,   90,    2, 0x08 /* Private */,
      15,    0,   93,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   11,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   11,
    QMetaType::Void, QMetaType::Bool,   11,
    QMetaType::Void,

       0        // eod
};

void ProcessWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ProcessWidget *_t = static_cast<ProcessWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->closeButtonClicked(); break;
        case 1: _t->nameChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->on_pauseBtn_clicked(); break;
        case 3: _t->on_startBtn_clicked(); break;
        case 4: _t->on_colorBtn_clicked(); break;
        case 5: _t->on_closeBtn_clicked(); break;
        case 6: _t->on_exportLogBtn_clicked(); break;
        case 7: _t->on_keepLogCheckBtn_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_resetLogBtn_clicked(); break;
        case 9: _t->on_drawCheckBtn_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->on_syncCheckBox_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->on_nameEdit_editingFinished(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ProcessWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ProcessWidget::closeButtonClicked)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ProcessWidget::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ProcessWidget::nameChanged)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ProcessWidget::staticMetaObject = {
    { &QGroupBox::staticMetaObject, qt_meta_stringdata_ProcessWidget.data,
      qt_meta_data_ProcessWidget,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *ProcessWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ProcessWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ProcessWidget.stringdata0))
        return static_cast<void*>(this);
    return QGroupBox::qt_metacast(_clname);
}

int ProcessWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGroupBox::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void ProcessWidget::closeButtonClicked()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void ProcessWidget::nameChanged(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
