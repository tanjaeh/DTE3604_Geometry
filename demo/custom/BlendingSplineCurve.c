#include "BlendingSplineCurve.h"

//Constructors and destructor
template <typename T>
//inline
BlendingSpline<T>::BlendingSpline(GMlib::PCurve<T, 3>* copy, int n, int wantedAnimation)
    : GMlib::PCurve<T,3>(0,0,0){
    _closed = copy->isClosed();
    _start = copy->getParStart();
    _end = copy->getParEnd();
    _wantedAnimation = wantedAnimation;
    _colorNum = wantedAnimation;

    std::cout << copy->getRadius(T(1)) << std::endl;


    //Change
    generateKnots(n+1);

    generateControlCurves(copy);
}


template <typename T>
BlendingSpline<T>::~BlendingSpline() {}


template <typename T>
bool BlendingSpline<T>::isClosed() const {
    return _closed;
}


template <typename T>
void BlendingSpline<T>::eval(T t, int d, bool left) const{
    //Next line must join all eval functions
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
    return _start;
}

template <typename T>
T BlendingSpline<T>::getEndP() const{
    return _end;
}


template <typename T>
void BlendingSpline<T>::localSimulate(double dt){
    switch (_wantedAnimation) {
    case 1:
        epitrochoid_anim();
        break;
    default:
        break;
    }


    this->resample();
    this->setEditDone();
}


template <typename T>
void BlendingSpline<T>::generateKnots(int n){
    _knotVector.clear();
    int n_ = n - (_closed ? 0 : 1);

    auto dt = (_end - _start) / n_;

    _knotVector.push_back(_start);
    _knotVector.push_back(_start);

    for (int i = 1; i < n_; i++) {
      _knotVector.push_back(_start + i * dt);
    }

    _knotVector.push_back(_end);
    _knotVector.push_back(_end);

    if (_closed) {
        _knotVector[0] = _start - dt;
        _knotVector[n_ + 2] = _end + dt;
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
void BlendingSpline<T>::showControlCurves() {
  for (GMlib::PCurve<T, 3>* curve : _controlCurves) {
    curve->setCollapsed(true);
    curve->sample(20, 0);
    curve->toggleDefaultVisualizer();
    curve->setColor(GMlib::GMcolor::blueViolet());
    this->insert(curve);
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


//Animation
template <typename T>
void BlendingSpline<T>::switchColor(){
    switch (_colorNum) {
    case 0:
        this->setColor(GMlib::GMcolor::red());
        _colorNum++;
        break;
    case 1:
        this->setColor(GMlib::GMcolor::orange());
        _colorNum++;
        break;
    case 2:
        this->setColor(GMlib::GMcolor::yellow());
        _colorNum++;
        break;
    case 3:
        this->setColor(GMlib::GMcolor::green());
        _colorNum++;
        break;
    case 4:
        this->setColor(GMlib::GMcolor::blue());
        _colorNum++;
        break;
    case 5:
        this->setColor(GMlib::GMcolor::purple());
        _colorNum++;
        break;
    case 6:
        this->setColor(GMlib::GMcolor::deepPink());
        _colorNum = 0;
        break;
    default:
        break;
    }

}

template <typename T>
void BlendingSpline<T>::epitrochoid_anim(){
    _controlCurves[0]->rotate(GMlib::Angle(_rot), GMlib::Vector<T,3>(1, 0, 0));
    _controlCurves[2]->rotate(GMlib::Angle(_rot), GMlib::Vector<T,3>(1, 0, 1));
    _controlCurves[4]->rotate(GMlib::Angle(_rot), GMlib::Vector<T,3>(-1, 0, 1));
    _controlCurves[6]->rotate(GMlib::Angle(_rot), GMlib::Vector<T,3>(1, 0, 0));
    _controlCurves[8]->rotate(GMlib::Angle(_rot), GMlib::Vector<T,3>(1, 0, 1));
    _controlCurves[10]->rotate(GMlib::Angle(_rot), GMlib::Vector<T,3>(-1, 0, 1));

    auto diag = (_move/0.005) * sqrt((_move*_move)/2);
    _controlCurves[1]->translate(GMlib::Vector<T,3>(diag, 0, diag));
    _controlCurves[3]->translate(GMlib::Vector<T,3>(0, 0, _move));
    _controlCurves[5]->translate(GMlib::Vector<T,3>(-diag, 0, diag));
    _controlCurves[7]->translate(GMlib::Vector<T,3>(-diag, 0, -diag));
    _controlCurves[9]->translate(GMlib::Vector<T,3>(0, 0, -_move));
    _controlCurves[11]->translate(GMlib::Vector<T,3>(diag, 0, -diag));



    _angle += _rot;
    if(fmod(_angle, M_PI/5) < 0.009){
        switchColor();
    }
    if(_angle > M_PI){
        _move = -_move;
        _angle = 0.0f;
    }
}
