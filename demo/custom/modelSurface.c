#include "modelsurface.h"

template <typename T>
ModelSurface<T>::ModelSurface(T const a){
    _a = a;
}

template <typename T>
ModelSurface<T>::~ModelSurface() {
}

template <typename T>
void ModelSurface<T>::eval(T u, T v, int d1, int d2, bool /*lu*/, bool /*lv*/) const {
  _p.setDim(d1 + 1, d2 + 1);

  T const x = _a * cos(u)*cos(u)*cos(u) * cos(v)*cos(v)*cos(v);
  T const x_du = -3*_a * sin(u) * cos(u)*cos(u) * cos(v)*cos(v)*cos(v);
  T const x_dv = -3*_a * cos(u)*cos(u)*cos(u) * sin(v) * cos(v)*cos(v);

  T const y = _a * sin(u)*sin(u)*sin(u) * cos(v)*cos(v)*cos(v);
  T const y_du = 3*_a * sin(u)*sin(u) * cos(u) * cos(v)*cos(v)*cos(v);
  T const y_dv = -3*_a * sin(u)*sin(u)*sin(u) * sin(v) * cos(v)*cos(v);

  T const z = _a * sin(v)*sin(v)*sin(v);
  T const z_du = 0;
  T const z_dv = 3*_a * sin(v)*sin(v) * cos(v);


  _p[0][0][0] = x;
  _p[0][0][1] = y;
  _p[0][0][2] = z;

  // Check derivation mehod
  if (_dm == GMlib::GM_DERIVATION_EXPLICIT) {
    if (d1) {
      _p[1][0][0] = x_du;
      _p[1][0][1] = y_du;
      _p[1][0][2] = z_du;
    }

    if (d2) {
      _p[0][1][0] = x_dv;
      _p[0][1][1] = y_dv;
      _p[0][1][2] = z_dv;
    }
  }
}

template <typename T>
bool ModelSurface<T>::isClosedU() const {
  return false;
}

template <typename T>
bool ModelSurface<T>::isClosedV() const {
  return false;
}

template <typename T>
T ModelSurface<T>::getStartPU() const {
  return T(0);
}

template <typename T>
T ModelSurface<T>::getStartPV() const {
  return T(0);
}

template <typename T>
T ModelSurface<T>::getEndPU() const {
  return M_PI;
}

template <typename T>
T ModelSurface<T>::getEndPV() const {
  return M_PI;
}
