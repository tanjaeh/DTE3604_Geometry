#ifndef MODELCURVE_H
#define MODELCURVE_H

#include "../../gmlib/modules/parametrics/gmpcurve.h"


template <typename T>
class ModelCurve : public GMlib::PCurve<T,3> {
    GM_SCENEOBJECT(ModelCurve)

public:
    ModelCurve( T radius = T(20) );
    ModelCurve(  T radius1, T radius2 );
    ModelCurve( const ModelCurve<T>& copy );
    virtual ~ModelCurve();

    // Public local functions
    T               getRadius(int i=1) const;
    void            setRadius( T radius = T(20) );
    void            setRadius( T radius1, T radius2);

    //****************************************
    //****** Virtual public functions   ******
    //****************************************

    // from PCurve
    bool            isClosed() const override;

protected:
    // Virtual functions from PCurve, which have to be implemented locally
    void            eval(T t, int d, bool l) const override;
    T               getStartP() const override;
    T               getEndP()   const override;

    void            computeSurroundingSphere( const std::vector<GMlib::DVector<GMlib::Vector<T,3>>>& /*p*/, GMlib::Sphere<T,3>& s ) const override;

    // Protected data for the curve
    T               _rx;
    T               _ry;

}; // END class modelCurve

// Include modelCurve class function implementations
#include "modelCurve.c"

#endif // MODELCURVE_H
