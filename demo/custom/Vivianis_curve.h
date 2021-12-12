#ifndef VIVIANIS_CURVE_H
#define VIVIANIS_CURVE_H
//#define _USE_MATH_DEFINES

#include "../../gmlib/modules/parametrics/gmpcurve.h"
//#include <math.h>

template <typename T>
class VivianisCurve : public GMlib::PCurve<T,3> {
    GM_SCENEOBJECT(VivianisCurve)

public:
    VivianisCurve(T radius);
    VivianisCurve(const VivianisCurve<T>& copy);
    virtual ~VivianisCurve();

//    T getRadius() const;
    bool isClosed() const override;


protected:
    void eval(T t, int d, bool left) const override;
    T getStartP() const override;
    T getEndP() const override;

    T _radius;


}; // END class VivianisCurve

#include "Vivianis_curve.c"

#endif // VIVIANIS_CURVE_H
