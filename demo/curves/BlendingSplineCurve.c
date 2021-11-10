#include "BlendingSplineCurve.h"

//Constructors and destructor
template <typename T>
inline
BlendingSpline<T>::BlendingSpline(GMlib::PCurve<T, 3>* copy, int n)
    : GMlib::PCurve<T,3>(0,0,0){
    _closed = copy->isClosed();
    _startKnotVector = copy->getParStart();
    _endKnotVector = copy->getParEnd();


    //Change
    generateKnots(n/*, _startKnotVector, _endKnotVector*/);

    generateControlCurves(copy);
}

//template <typename T>
//inline
//BlendingSpline<T>::BlendingSpline( const ClosedSubDivisionCurve<T>& copy )
//    : PCurve<T,3>(copy){}



template <typename T>
BlendingSpline<T>::~BlendingSpline() {}


template <typename T>
bool BlendingSpline<T>::isClosed() const {
    return _closed;
}


template <typename T>
T BlendingSpline<T>::getStartP() const{
    return _startKnotVector;
}

template <typename T>
T BlendingSpline<T>::getEndP() const{
    return _endKnotVector;
}


template <typename T>
void BlendingSpline<T>::generateKnots(int n/*, T start, T end*/){
    _knotVector.clear();

    auto n_unique = n - (_closed ? 0 : 1);

    auto dt = (end - start) / n_unique;


    // Total of 3 starts and 3 ends are added
    _knotVector.push_back(_startKnotVector);
    _knotVector.push_back(_startKnotVector);

    for (int i = start ; i <= (n-2) ; i++){
        _knotVector.push_back(_startKnotVector + (i * dt));
    }

    _knotVector.push_back(_endKnotVector);
    _knotVector.push_back(_endKnotVector);


//    if(_closed){
//        _knotVector[0] = _startKnotVector - dt;
//        _knotVector[n_unique + 2] = _endKnotVector + dt;
//    }
}


template <typename T>
void BlendingSpline::generateControlCurves(GMlib::PCurve<T,3>* curve){
    int n = _knotVector.size() - 2;
    _controlCurves.resize(n);


    for(int i = 0 ; i < n ; i++){
        _controlCurves[i] = new GMlib::PSubCurve<T>(curve, _knotVector[i], _knotVector[i+2], _knotVector[i+1]);
    }


    if(_closed){ // If closed, start and end is the same
        _controlCurves[n-1] = _controlCurves[0];
    }

}



