#include "Vivianis_curve.h"

template <typename T>
inline
VivianisCurve<T>::VivianisCurve(T radius) : GMlib::PCurve<T,3>(20, 0, 0){
    _radius = radius;
}

template <typename T>
inline
VivianisCurve<T>::VivianisCurve(const VivianisCurve<T> &copy) : GMlib::PCurve<T,3>(copy){}

template <typename T>
VivianisCurve<T>::~VivianisCurve() {}



template <typename T>
bool VivianisCurve<T>::isClosed() const {
    return true;
}

template <typename T>
void VivianisCurve<T>::eval(T t, int d, bool left) const{
    //Next line must join all eval functions
    this->_p.setDim( d + 1 );


    this->_p[0][0] = _radius * cos(t) * cos(t);
    this->_p[0][1] = _radius * cos(t) * sin(t);
    this->_p[0][2] = _radius * sin(t);
}


template <typename T>
T VivianisCurve<T>::getStartP() const{
    return -M_PI;
}

template <typename T>
T VivianisCurve<T>::getEndP() const{
    return M_PI;
}
