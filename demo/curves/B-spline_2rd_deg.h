#ifndef BSPLINE_2RD_DEG_H
#define BSPLINE_2RD_DEG_H


#include "../../gmlib/modules/parametrics/gmpcurve.h"

#include <algorithm>


template <typename T>
class BSpline : public GMlib::PCurve<T,3> {
    GM_SCENEOBJECT(BSpline)

public:
    BSpline( const GMlib::DVector<GMlib::Vector<T,3>>& c, bool closed = false);
    BSpline( const GMlib::DVector<GMlib::Vector<T,3>>& p, int n, bool closed = false);
    BSpline( const BSpline<T>& copy );
    virtual ~BSpline();


    bool            isClosed() const override;

protected:
    // Virtual functions from PCurve, which have to be implemented locally
    void            eval(T t, int d, bool l) const override;
    T               getStartP() const override;
    T               getEndP()   const override;


private:
    void generateKnotVector(int n);
    T W(T t, int i, int d) const;
    GMlib::Vector<T,3> B(T t, int i) const;
    int getIndex(T t);

//    int n; //Control point
//    int d = 2;
//    T start, end;
    GMlib::DVector<GMlib::Vector<T,3>> _c; //Control points
    std::vector<T> _knotVector;
    bool _closed;


}; // END class BSpline

// Include BSpline class function implementations
#include "B-spline_2rd_deg.c"


#endif // BSPLINE_2RD_DEG_H
