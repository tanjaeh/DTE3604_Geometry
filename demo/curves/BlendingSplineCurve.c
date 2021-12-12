#include "BlendingSplineCurve.h"

//Constructors and destructor
template <typename T>
//inline
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
void BlendingSpline<T>::eval(T t, int d, bool left) const{
    this->_p.setDim(d + 1);

    auto i = getIndex(t);

    auto b = B(t, i);

    auto ci_1 = getControlPoint(i - 1, t);
    auto ci = getControlPoint(i, t);

    //this->_p[0] = b * GMlib::Vector<T,2>(ci_1, ci);
    this->_p[0] = b[0] * ci_1 + b[1] * ci;
    // c(t) = (1 - Bow1, Bow1) * (ci_1, ci)
//    this->_p[0] = ci_1 + b[1] * (ci_1 - ci);
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
void BlendingSpline<T>::localSimulate(double dt){

}

template <typename T>
void BlendingSpline<T>::showControlCurves() {
  for (GMlib::PCurve<T, 3>* curve : _controlCurves) {
    curve->setCollapsed(true);
    curve->sample(60, 0);
    curve->toggleDefaultVisualizer();
    curve->setColor(GMlib::GMcolor::blueViolet());
    this->insert(curve);
  }
}




template <typename T>
void BlendingSpline<T>::generateKnots(int n/*, T start, T end*/){
    _knotVector.clear();

    auto n_unique = n - (_closed ? 0 : 1);

    auto dt = (_endKnotVector - _startKnotVector) / n_unique;


    // Total of 3 starts and 3 ends are added
    _knotVector.push_back(_startKnotVector);
    _knotVector.push_back(_startKnotVector);

    for (int i = _startKnotVector ; i <= n_unique ; i++){
        _knotVector.push_back(_startKnotVector + (i * dt));
    }

    _knotVector.push_back(_endKnotVector);
    _knotVector.push_back(_endKnotVector);


    if(_closed){ //Closes the start/end gap
        _knotVector[0] = _startKnotVector - dt;
        _knotVector[n_unique + 2] = _endKnotVector + dt;
    }
}


template <typename T>
void BlendingSpline<T>::generateControlCurves(GMlib::PCurve<T,3>* curve){
    int n = _knotVector.size() - 2;
    _controlCurves.resize(n);


    for(int i = 0 ; i < n ; i++){
        _controlCurves[i] = new GMlib::PSubCurve<T>(curve, _knotVector[i], _knotVector[i+2], _knotVector[i+1]);
    }


    if(_closed){ // If closed, start and end is the same
        _controlCurves[n-1] = _controlCurves[0];
    }

}


template <typename T>
int BlendingSpline<T>::getIndex(T t) const{
    if (t >= this->getEndP()){
        return _knotVector.size() - 3;
    }
    else{
        return std::distance(_knotVector.begin(), std::upper_bound(_knotVector.begin(), _knotVector.end(), t)) -1;
    }
}


template <typename T>
T BlendingSpline<T>::W(T t, int i, int d) const{
    return (t - _knotVector[i]) / (_knotVector[i+d] - _knotVector[i]);
}

template <typename T>
GMlib::Vector<T, 2> BlendingSpline<T>::B(T t, int i) const{
    T Bow1 = B_polynomial_function(W(t, i, 1));

    return GMlib::Vector<T,2>(1 - Bow1, Bow1);
}

template <typename T>
T BlendingSpline<T>::B_polynomial_function(T t) const {
    return 3*t*t - 2*t*t*t;
}

template <typename T>
GMlib::Point<T,3> BlendingSpline<T>::getControlPoint(int i, T t) const{
    T wt = W(t, i, 2);
    T ct = _controlCurves[i]->getParEnd() * wt + _controlCurves[i]->getParStart() * (1 - wt);

    return _controlCurves[i]->evaluateParent(ct, 0)[0];
}

