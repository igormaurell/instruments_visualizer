QT       -= core gui

TARGET = VisionSensors
TEMPLATE = lib
CONFIG += staticlib
CONFIG += C++11

INCLUDEPATH += -I/usr/include/eigen3
#INCLUDEPATH += $$PWD/../../ExternalLibraries/eigen-eigen-3.2.9
INCLUDEPATH += $$PWD/../../Libs/CsvUtil
#INCLUDEPATH += $$PWD/../../ExternalLibraries/CsvUtil
INCLUDEPATH += -I/usr/local/include/opencv
#INCLUDEPATH += $$PWD/../../ExternalLibraries/OpenCV3.1/include
INCLUDEPATH += $$PWD/../../Core/Vision

HEADERS += VisionSensors.h \
    AnalogMeterDetectorCLT.h \
    LevelTracker.h \
    PresenceTracker.h \
    DiffractionLevelTracker.h \
    AnalogMeterTrackerPC.h \
    MovementWrap.h \
    LevelTrackerAuto.h \
    KalmanWrap.h \
    TemperatureTrackerThermo.h \
    TemperatureTrackerTCam.h \
    LuminanceWrap.h

SOURCES += \
    AnalogMeterDetectorCLT.cpp \
    LevelTracker.cpp \
    PresenceTracker.cpp \
    DiffractionLevelTracker.cpp \
    AnalogMeterTrackerPC.cpp \
    LevelTrackerAuto.cpp \
    TemperatureTrackerThermo.cpp \
    TemperatureTrackerTCam.cpp

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../VisionImplementationCv/release/ -lVisionImplementationCv
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../VisionImplementationCv/debug/ -lVisionImplementationCv
else:unix:!macx: LIBS += -L$$OUT_PWD/../VisionImplementationCv/ -lVisionImplementationCv

LIBS += `pkg-config --libs opencv`

INCLUDEPATH += $$PWD/../VisionImplementationCv
DEPENDPATH += $$PWD/../VisionImplementationCv

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../VisionImplementationCv/release/libVisionImplementationCv.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../VisionImplementationCv/debug/libVisionImplementationCv.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../VisionImplementationCv/release/VisionImplementationCv.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../VisionImplementationCv/debug/VisionImplementationCv.lib
else:unix:!macx: PRE_TARGETDEPS += $$OUT_PWD/../VisionImplementationCv/libVisionImplementationCv.a
