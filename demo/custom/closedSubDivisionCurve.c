#include <cmath>
#include "closedSubDivisionCurve.h"

//Constructors and destructor
template <typename T>
inline
ClosedSubDivisionCurve<T>::ClosedSubDivisionCurve(GMlib::DVector<GMlib::Vector<T,3>> p, int d)
    : GMlib::PCurve<T,3>(0, 0, 0), _p{p}, _d{d} {}

template <typename T>
inline
ClosedSubDivisionCurve<T>::ClosedSubDivisionCurve( const ClosedSubDivisionCurve<T>& copy )
    : PCurve<T,3>(copy){}


//The destructor clean up and destroy all private data
template <typename T>
ClosedSubDivisionCurve<T>::~ClosedSubDivisionCurve() {}


template <typename T>
bool ClosedSubDivisionCurve<T>::isClosed() const {
    return true;
}

template <typename T>
void ClosedSubDivisionCurve<T>::resample(std::vector<GMlib::DVector<GMlib::Vector<T, 3>>> &p, GMlib::Sphere<T, 3> &s, const std::vector<T> &t, int d) const {
    int n = _p.getDim();
    int k = this->_visu.no_sample;
    int m = pow(2, k) * n + 1;

    p.resize(m);
    s.reset();

    for (int i = 0 ; i < p.size() ; i++){
        p[i].setDim(1);
    }

    for (int i = 0; i < n; i++) {
      p[i][0] = _p[i];
    }
    p[n][0] = _p[0];

    for (int i = 0; i < k; i++) {
      doublingPoints(p, n);
      for(int j = 0 ; j < _d ; j++ ){
          smoothing(p, n);
      }

    }

    computeSurroundingSphere(p, s);
    if (d > _der_implemented || (d > 0 && this->_dm == GMlib::GM_DERIVATION_DD)){
        GMlib::DD::compute1D(p, t, isClosed(), d, _der_implemented);
    }

}



// Protected virtual functions from PCurve
template <typename T>
void ClosedSubDivisionCurve<T>::eval( T t, int d, bool l ) const {
    //Should be empty
}


//Custom functions
template <typename T>
void ClosedSubDivisionCurve<T>::smoothing(std::vector<GMlib::DVector<GMlib::Vector<T, 3>>>& p, int no_elements) const {
    for(int i = 0 ; i < no_elements ; i++){
        p[i][0] = (p[i][0] + p[i+1][0])/T(2);
    }
    p[no_elements][0] = p[0][0];
}

template <typename T>
void ClosedSubDivisionCurve<T>::doublingPoints(std::vector<GMlib::DVector<GMlib::Vector<T, 3>>>& p, int & no_elements) const {
    for(int i = no_elements ; i > 0 ; i--){
        p[i*2][0] = p[i][0];

        p[(i*2) - 1][0] = (p[i][0] + p[i-1][0])/ T(2);
    }


    no_elements = no_elements * 2;
}





