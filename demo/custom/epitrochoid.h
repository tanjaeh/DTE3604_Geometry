#ifndef EPITROCHOID_H
#define EPITROCHOID_H

#include "../../gmlib/modules/parametrics/gmpcurve.h"

template <typename T>
class Epitrochid : public GMlib::PCurve<T,3> {
    GM_SCENEOBJECT(Epitrochid)

public:
    Epitrochid( T radius = T(10));
    Epitrochid(T r1, T r2);
    Epitrochid(const Epitrochid<T>& copy);

    virtual ~Epitrochid();

    bool isClosed() const override;

protected:
    void eval(T t, int d, bool left) const override;
    T getStartP() const override;
    T getEndP() const override;

    void computeSurroundingSphere( const std::vector<GMlib::DVector<GMlib::Vector<T,3>>>& /*p*/, GMlib::Sphere<T,3>& s ) const override;


    T _rx;
    T _rw;
}; //END class Epitrochid

#include "epitrochoid.c"

#endif // EPITROCHOID_H
