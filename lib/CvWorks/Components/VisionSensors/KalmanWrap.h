#ifndef KALMANWRAP_H
#define KALMANWRAP_H
#include "VisionSensors.h"

namespace VisionSense {

//http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/

template<class TImg>
class KalmanWrap : public VisionCore::Tracker<TImg,double>
{
public:
    KalmanWrap(VisionCore::Tracker<TImg,double>* sensor);
    virtual ~KalmanWrap();

    void update(const VisionCore::Frame<TImg>& frame);

    const double& getLastTrack();

    bool lostTrack() const;

    double q; //process noise covariance
    double r; //measurement noise covariance

private:
    VisionCore::Tracker<TImg,double>* baseSensor;



    double x; //value
    double p; //estimation error covariance
    double k; //kalman gain
    bool initialized;
};

template<class TImg>
KalmanWrap<TImg>::KalmanWrap(VisionCore::Tracker<TImg,double>* sensor)
    :baseSensor(sensor)
    ,q(0.01)
    ,r(0.01)
    ,p(0)
    ,initialized(false)
{

}

template<class TImg>
KalmanWrap<TImg>::~KalmanWrap()
{
    if(baseSensor)
        delete baseSensor;
}

template<class TImg>
void KalmanWrap<TImg>::update(const VisionCore::Frame<TImg> &frame)
{
    //Update base sensor
    baseSensor->update(frame);
    double measurement = baseSensor->getLastTrack();

    if(!initialized){
        x=measurement;
        initialized=true;
    }
    //prediction update
    //omit x = x
    p = p + q;

    //measurement update
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1 - k) * p;
}

template<class TImg>
const double &KalmanWrap<TImg>::getLastTrack()
{
    return x;

}

template<class TImg>
bool KalmanWrap<TImg>::lostTrack() const
{
    return baseSensor->lostTrack();
}

}
#endif // KALMANWRAP_H
