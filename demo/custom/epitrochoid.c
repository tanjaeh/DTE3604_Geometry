#include "epitrochoid.h"

template <typename T>
inline
Epitrochid<T>::Epitrochid(T radius) : GMlib::PCurve<T,3>(10, 0, 0){
    _rx = radius;
    _rw = radius;
}


template <typename T>
inline
Epitrochid<T>::Epitrochid(T r1, T r2) : GMlib::PCurve<T,3>(10, 0, 0){
    _rx = r1;
    _rw = r2;
}

template <typename T>
inline
Epitrochid<T>::Epitrochid( const Epitrochid<T>& copy ) : PCurve<T,3>(copy) {}


template <typename T>
Epitrochid<T>::~Epitrochid(){}




template <typename T>
bool Epitrochid<T>::isClosed() const {
    return true;
}


template <typename T>
void Epitrochid<T>::eval( T t, int d, bool /*l*/ ) const { //bool l tells if it eval from left (default True)
    //Next line must join all eval functions
    this->_p.setDim( d + 1 );


    auto b_x = _rx/6;
    auto b_w = _rw/6;
    auto h = b_x * 3;

    const T ct_x = (_rx + b_x) * cos(t) - h * cos((_rx/b_x + 1) * t);
    const T st_w = (_rw + b_w) * sin(t) - h * sin((_rw/b_w + 1) * t);

    this->_p[0][0] = ct_x;
    this->_p[0][1] = T(0);
    this->_p[0][2] = st_w;
}


template <typename T>
T Epitrochid<T>::getStartP() const {
    return T(0);
}


template <typename T>
T Epitrochid<T>::getEndP()const {
    return T( M_2PI );
}


template <typename T>
void Epitrochid<T>::computeSurroundingSphere( const std::vector<GMlib::DVector<GMlib::Vector<T,3>>>& /*p*/, GMlib::Sphere<T,3>& s ) const {

    s.resetPos(GMlib::Point<T,3>(T(0)));
    if(_rx > _rw)
        s.resetRadius(_rx);
    else
        s.resetRadius(_rw);
}
