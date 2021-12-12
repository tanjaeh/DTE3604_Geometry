#ifndef BSPLINE_2RD_DEG_H
#define BSPLINE_2RD_DEG_H


#include "../../gmlib/modules/parametrics/gmpcurve.h"
#include "../../gmlib/modules/parametrics/surfaces/gmpsphere.h"

#include <algorithm>


template <typename T>
class BSpline : public GMlib::PCurve<T,3> {
    GM_SCENEOBJECT(BSpline)

public:
    BSpline( const GMlib::DVector<GMlib::Vector<T,3>>& c, bool blend, bool closed = false);
    BSpline( const GMlib::DVector<GMlib::Vector<T,3>>& p, int n, bool closed = false);
    BSpline( const BSpline<T>& copy );
    virtual ~BSpline();


    bool            isClosed() const override;
    void            showControlPoints();

protected:
    // Virtual functions from PCurve, which have to be implemented locally
    void            eval(T t, int d, bool l) const override;
    T               getStartP() const override;
    T               getEndP()   const override;


private:
    void generateKnotVector(int n, T start, T end);
    void generateControlPoints( const GMlib::DVector<GMlib::Vector<T,3>>& p, int n);
    T W(T t, int i, int d) const;
    GMlib::Vector<T,3> B(T t, int i) const;
    GMlib::Vector<T,3> Blend(T t, int i) const;
    T B_polynomial_function(T t) const;
    int getIndex(T t) const;
    std::vector<T> getXiVector(const GMlib::DVector<GMlib::Vector<T,3>>& p);

    GMlib::DVector<GMlib::Vector<T,3>> _c; //Control points
    std::vector<T> _knotVector;
    std::vector<T> _xi;
    bool _closed;
    bool _blend;


}; // END class BSpline

// Include BSpline class function implementations
#include "B-spline_2rd_deg.c"


#endif // BSPLINE_2RD_DEG_H
