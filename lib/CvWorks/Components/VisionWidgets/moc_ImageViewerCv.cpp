/****************************************************************************
** Meta object code from reading C++ file 'ImageViewerCv.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "ImageViewerCv.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ImageViewerCv.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ImageViewerCv_t {
    QByteArrayData data[14];
    char stringdata0[117];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ImageViewerCv_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ImageViewerCv_t qt_meta_stringdata_ImageViewerCv = {
    {
QT_MOC_LITERAL(0, 0, 13), // "ImageViewerCv"
QT_MOC_LITERAL(1, 14, 11), // "imageRedraw"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 9), // "QPainter*"
QT_MOC_LITERAL(4, 37, 7), // "painter"
QT_MOC_LITERAL(5, 45, 9), // "leftClick"
QT_MOC_LITERAL(6, 55, 5), // "point"
QT_MOC_LITERAL(7, 61, 8), // "setImage"
QT_MOC_LITERAL(8, 70, 7), // "cv::Mat"
QT_MOC_LITERAL(9, 78, 3), // "img"
QT_MOC_LITERAL(10, 82, 8), // "setScale"
QT_MOC_LITERAL(11, 91, 6), // "scale_"
QT_MOC_LITERAL(12, 98, 12), // "setShowImage"
QT_MOC_LITERAL(13, 111, 5) // "value"

    },
    "ImageViewerCv\0imageRedraw\0\0QPainter*\0"
    "painter\0leftClick\0point\0setImage\0"
    "cv::Mat\0img\0setScale\0scale_\0setShowImage\0"
    "value"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ImageViewerCv[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   39,    2, 0x06 /* Public */,
       5,    1,   42,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   45,    2, 0x0a /* Public */,
      10,    1,   48,    2, 0x0a /* Public */,
      12,    1,   51,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QPoint,    6,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void, QMetaType::Int,   11,
    QMetaType::Void, QMetaType::Bool,   13,

       0        // eod
};

void ImageViewerCv::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ImageViewerCv *_t = static_cast<ImageViewerCv *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->imageRedraw((*reinterpret_cast< QPainter*(*)>(_a[1]))); break;
        case 1: _t->leftClick((*reinterpret_cast< QPoint(*)>(_a[1]))); break;
        case 2: _t->setImage((*reinterpret_cast< const cv::Mat(*)>(_a[1]))); break;
        case 3: _t->setScale((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->setShowImage((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ImageViewerCv::*)(QPainter * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ImageViewerCv::imageRedraw)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ImageViewerCv::*)(QPoint );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ImageViewerCv::leftClick)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ImageViewerCv::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_ImageViewerCv.data,
      qt_meta_data_ImageViewerCv,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *ImageViewerCv::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ImageViewerCv::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ImageViewerCv.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int ImageViewerCv::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void ImageViewerCv::imageRedraw(QPainter * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ImageViewerCv::leftClick(QPoint _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
