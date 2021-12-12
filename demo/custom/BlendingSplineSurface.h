#ifndef BLENDINGSPLINESURFACE_H
#define BLENDINGSPLINESURFACE_H

#include "../../gmlib/modules/parametrics/gmpsurf.h"
#include "../../gmlib/modules/parametrics/surfaces/gmpsubsurf.h"
#include "Helpers/subpatch.h"

template <typename T>
class BlendingSurface : public GMlib::PSurf<T,3> {
    GM_SCENEOBJECT(BlendingSurface)

public:
    BlendingSurface(GMlib::PSurf<T,3>* copy, int u, int v, int uSample, int vSample);
//    BlendingSurface(const BlendingSurface<T>& copy);
    virtual ~BlendingSurface();

    bool isClosedU() const override;
    bool isClosedV() const override;

    void showControlSurf();
    void edit(GMlib::SceneObject *lp) override;

protected:
    T getStartPU() const override;
    T getStartPV() const override;

    T getEndPU() const override;
    T getEndPV() const override;

    void eval(T u, T v, int d1, int d2, bool left_u = true, bool left_v = true) const override;

private:
    void generateKnotVector(int n, bool isU, std::vector<T>& knotVector);
    void generateContorlSurf();

    T getIndex(T t, bool isU) const;

    T W(T t, int i, int d, std::vector<T> const& knotVector) const;
    GMlib::Vector<T,2> blend(T t) const;


    bool _closedU;
    bool _closedV;

    T _startPU;
    T _startPV;

    T _endPU;
    T _endPV;

    int _sampleU;
    int _sampleV;


    GMlib::PSurf<T,3>* _surface;

    std::vector<T> _U_knotVector;
    std::vector<T> _V_knotVector;

    GMlib::DMatrix<GMlib::SubPatch<T>*> _subSurfs;




}; //END BlendingSurface Class

#include "BlendingSplineSurface.c"

#endif // BLENDINGSPLINESURFACE_H
