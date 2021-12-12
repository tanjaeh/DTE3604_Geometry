#ifndef BLENDINGSPLINECURVE_H
#define BLENDINGSPLINECURVE_H

#include "../../gmlib/modules/parametrics/gmpcurve.h"
#include "../../gmlib/modules/parametrics/curves/gmpsubcurve.h"

enum AnimationCycle{
    Growing,
    Shrinking,
    Flying
};


template <typename T>
class BlendingSpline : public GMlib::PCurve<T, 3>{
    GM_SCENEOBJECT(BlendingSpline)

public:
    BlendingSpline(GMlib::PCurve<T, 3>* copy, int n, int wantedAnimation = 0);
//    BlendingSpline(const BlendingSpline<T>& copy);
    virtual ~BlendingSpline();

    bool isClosed() const override;

    void showControlCurves();


protected:
    T getStartP() const override;
    T getEndP() const override;

    void eval(T t, int d, bool left) const override;

    void localSimulate(double dt) override;


private:
     void generateKnots(int n/*, T start = T(0), T end = T(1)*/);

     void generateControlCurves(GMlib::PCurve<T,3>* curve);

     int getIndex(T t) const;

     T W(T t, int i, int d) const;
     GMlib::Vector<T,2> B(T t, int i) const;
     T B_polynomial_function(T t) const;

     GMlib::Point<T,3> getControlPoint(int i, T t) const;

     //animations
     void switchColor();
     void epitrochoid_anim();
     void astroid_anim();
     void changeAnimation();
     GMlib::Vector<float, 3> circleMovement(float r, float currentAngle);
     int _wantedAnimation;
     int _colorNum = 0;
     float _angle = 0.0f;
     T _move = 0.005;
     double _rot = 0.01;

     AnimationCycle current_animation = Growing;


     std::vector< GMlib::PSubCurve<T>* > _controlCurves;
     std::vector<T> _knotVector;
     bool _closed;

     T _start;
     T _end;


}; //End BlendingSpline Class

#include "BlendingSplineCurve.c"

#endif // BLENDINGSPLINECURVE_H
