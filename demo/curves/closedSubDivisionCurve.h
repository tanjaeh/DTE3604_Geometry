#ifndef CLOSEDSUBDIVISIONCURVE_H
#define CLOSEDSUBDIVISIONCURVE_H

#include "../../gmlib/modules/parametrics/gmpcurve.h"

template <typename T>
class ClosedSubDivisionCurve : public GMlib::PCurve<T,3>{
    GM_SCENEOBJECT(ClosedSubDivisionCurve)

public:
    ClosedSubDivisionCurve(GMlib::DVector<GMlib::Vector<T,3>> p, int d);
    ClosedSubDivisionCurve( const ClosedSubDivisionCurve<T>& copy );
    virtual ~ClosedSubDivisionCurve();

    bool isClosed() const override;

    void resample(std::vector<GMlib::DVector<GMlib::Vector<T, 3>>> &p, GMlib::Sphere<T, 3> &s, const std::vector<T> &t, int d) const override;

protected:
    void eval(T t, int d, bool l) const override;
    T getStartP() const override { return 0; }
    T getEndP() const override { return 1; }

private:
    void smoothing(std::vector<GMlib::DVector<GMlib::Vector<T, 3>>>& p, int no_elements) const; //Glatting
    void doublingPoints(std::vector<GMlib::DVector<GMlib::Vector<T, 3>>>& p, int & no_elements) const;

    int _d; //Degree
    GMlib::DVector<GMlib::Vector<T,3>> _p; //points

}; // END class closedSubDivisionCurve

#include "closedSubDivisionCurve.c"


#endif // CLOSEDSUBDIVISIONCURVE_H
