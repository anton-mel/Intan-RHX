/****************************************************************************
** Meta object code from reading C++ file 'pipelinedatarhxcontroller.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.9.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Engine/API/Synthetic/pipelinedatarhxcontroller.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pipelinedatarhxcontroller.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.9.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN25PipelineDataRHXControllerE_t {};
} // unnamed namespace

template <> constexpr inline auto PipelineDataRHXController::qt_create_metaobjectdata<qt_meta_tag_ZN25PipelineDataRHXControllerE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "PipelineDataRHXController",
        "onTCPConnected",
        "",
        "onTCPDisconnected",
        "onTCPError",
        "QAbstractSocket::SocketError",
        "error",
        "onTCPDataReceived",
        "onReconnectTimer"
    };

    QtMocHelpers::UintData qt_methods {
        // Slot 'onTCPConnected'
        QtMocHelpers::SlotData<void()>(1, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'onTCPDisconnected'
        QtMocHelpers::SlotData<void()>(3, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'onTCPError'
        QtMocHelpers::SlotData<void(QAbstractSocket::SocketError)>(4, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 5, 6 },
        }}),
        // Slot 'onTCPDataReceived'
        QtMocHelpers::SlotData<void()>(7, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'onReconnectTimer'
        QtMocHelpers::SlotData<void()>(8, 2, QMC::AccessPrivate, QMetaType::Void),
    };
    QtMocHelpers::UintData qt_properties {
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<PipelineDataRHXController, qt_meta_tag_ZN25PipelineDataRHXControllerE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject PipelineDataRHXController::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN25PipelineDataRHXControllerE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN25PipelineDataRHXControllerE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN25PipelineDataRHXControllerE_t>.metaTypes,
    nullptr
} };

void PipelineDataRHXController::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<PipelineDataRHXController *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->onTCPConnected(); break;
        case 1: _t->onTCPDisconnected(); break;
        case 2: _t->onTCPError((*reinterpret_cast< std::add_pointer_t<QAbstractSocket::SocketError>>(_a[1]))); break;
        case 3: _t->onTCPDataReceived(); break;
        case 4: _t->onReconnectTimer(); break;
        default: ;
        }
    }
}

const QMetaObject *PipelineDataRHXController::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PipelineDataRHXController::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN25PipelineDataRHXControllerE_t>.strings))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "AbstractRHXController"))
        return static_cast< AbstractRHXController*>(this);
    return QObject::qt_metacast(_clname);
}

int PipelineDataRHXController::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 5;
    }
    return _id;
}
QT_WARNING_POP
