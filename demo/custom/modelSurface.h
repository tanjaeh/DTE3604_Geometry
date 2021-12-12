#ifndef MODELSURFACE_H
#define MODELSURFACE_H

#include <parametrics/gmpsurf.h>

template <typename T>
class ModelSurface : public GMlib::PSurf<T, 3> {
  GM_SCENEOBJECT(ModelSurface)
public:
  ModelSurface(T const a);

  ~ModelSurface() override;

  bool isClosedU() const override;
  bool isClosedV() const override;

protected:
  void eval(T u, T v, int d1, int d2, bool lu, bool lv) const override;

  T getStartPU() const override;
  T getStartPV() const override;
  T getEndPU() const override;
  T getEndPV() const override;

private:
  T _a;
};

#include "modelsurface.c"

#endif   // MODELSURFACE_H
