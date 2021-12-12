#include "BlendingSplineSurface.h"

//Public
template <typename T>
inline
BlendingSurface<T>::BlendingSurface(GMlib::PSurf<T,3> *copy, int u, int v, int uSample, int vSample) : _surface(copy) {
    _closedU = copy->isClosedU();
    _closedV = copy->isClosedV();

    _startPU = copy->getParStartU();
    _startPV = copy->getParStartV();

    _endPU = copy->getParEndU();
    _endPV = copy->getParEndV();

    _sampleU = uSample;
    _sampleV = vSample;



    generateKnotVector(u, true, _U_knotVector);
    generateKnotVector(v, false, _V_knotVector);

    generateContorlSurf();
}

template <typename T>
BlendingSurface<T>::~BlendingSurface() {}

template <typename T>
bool BlendingSurface<T>::isClosedU() const {
    return _closedU;
}

template <typename T>
bool BlendingSurface<T>::isClosedV() const {
    return _closedV;
}

template <typename T>
void BlendingSurface<T>::showControlSurf(){
    int const i_lim = _U_knotVector.size() - 2 - _closedU;
    int const j_lim = _V_knotVector.size() - 2 - _closedV;

    for(int i = 0 ; i < i_lim ; i++){
        for(int j = 0 ; j < j_lim ; j++){
            auto surf = _subSurfs[i][j];
            surf->setCollapsed(true);
            this->insert(surf);

        }
    }
}

template <typename T>
void BlendingSurface<T>::edit(GMlib::SceneObject* /*lp*/) {
  setEditDone();
  sample(_sampleU, _sampleV);
}



//Protected methods
template <typename T>
T BlendingSurface<T>::getStartPU() const {
    return _startPU;
}

template <typename T>
T BlendingSurface<T>::getStartPV() const {
    return _startPV;
}


template <typename T>
T BlendingSurface<T>::getEndPU() const {
    return _endPU;
}

template <typename T>
T BlendingSurface<T>::getEndPV() const {
    return _endPV;
}


template <typename T>
void BlendingSurface<T>::eval(T u, T v, int d1, int d2, bool left_u, bool left_v) const{
    this->_p.setDim(d1 + 1, d2 + 1);

    int indexU = getIndex(u, true);
    int indexV = getIndex(v, false);

    // Page 246 of compendium
    GMlib::HqMatrix<float, 3> A_ij = _subSurfs(indexU - 1)(indexV - 1)->getMatr();
    GMlib::HqMatrix<float, 3> A_i1_j = _subSurfs(indexU)(indexV - 1)->getMatr();
    GMlib::HqMatrix<float, 3> A_i_j1 = _subSurfs(indexU - 1)(indexV)->getMatr();
    GMlib::HqMatrix<float, 3> A_i1_j1 = _subSurfs(indexU)(indexV)->getMatr();

    // D = delta
    GMlib::HqMatrix<float, 3> D_i_A = A_i1_j - A_ij;
    GMlib::HqMatrix<float, 3> D_j_A = A_i_j1 - A_ij;
    GMlib::HqMatrix<float, 3> D_ij_A = A_i1_j1 - A_i_j1 - A_i1_j + A_ij;


    GMlib::Vector<T,2> b_u = blend( W(u, indexU, 1, _U_knotVector) );
    GMlib::Vector<T,2> b_v = blend( W(v, indexV, 1, _V_knotVector) );

    GMlib::HqMatrix<T,3> M = A_ij + D_i_A * b_u[0] + D_j_A * b_v[0] + D_ij_A * (b_u[0] * b_v[0]);
    GMlib::HqMatrix<T,3> Mu = D_i_A * b_u[1] + D_ij_A * (b_u[1] * b_v[0]);
    GMlib::HqMatrix<T,3> Mv = D_j_A * b_v[1] + D_ij_A * (b_u[0] * b_v[1]);


    GMlib::DMatrix<GMlib::Vector<T,3>> S = _surface->evaluate(u, v, d1, d2);


    GMlib::Point<T,3> p = S[0][0];
    GMlib::Vector<T,3> Su = S[1][0];
    GMlib::Vector<T,3> Sv = S[0][1];

    this->_p[0][0] = M * p;
    this->_p[1][0] = Mu * p + M * Su;
    this->_p[0][1] = Mv * p + M * Sv;
}


//Private
template <typename T>
void BlendingSurface<T>::generateKnotVector(int n, bool isU, std::vector<T>& knotVector){
    T start;
    T end;
    T parDelta;
    bool closed;
    if(isU){
        start = _startPU;
        end = _endPU;
        closed = _closedU;
        parDelta = _surface->getParDeltaU();
    }
    else{
        start = _startPV;
        end = _endPV;
        closed = _closedV;
        parDelta = _surface->getParDeltaV();
    }

    int n_ = n - (closed ? 0 : 1);

    float dt = parDelta / n_;

    knotVector.push_back(start);
    knotVector.push_back(start);

    for (int i = 1 ; i < n_ ; i++) {
        T const t = start + i * dt;
        knotVector.push_back(t);
    }

    knotVector.push_back(end);
    knotVector.push_back(end);

    if (closed) {
        knotVector[0] = start - dt;
        knotVector[n_ + 2] = end + dt;
    }
}

template <typename T>
void BlendingSurface<T>::generateContorlSurf(){
    int u_dim = _U_knotVector.size() - _closedU;
    int v_dim = _V_knotVector.size() - _closedV;
    _subSurfs = GMlib::DMatrix<GMlib::SubPatch<T>*>(u_dim, v_dim);

    // Same as computing 'n' in blendingspline
    // Page 229 for indices/limits
    int const i_lim = _U_knotVector.size() - 2 - _closedU;
    int const j_lim = _V_knotVector.size() - 2 - _closedV;


    for(int i = 0 ; i < i_lim ; i++) {
        for(int j = 0 ; j < j_lim ; j++) {
            // Page 245: we get for i = 1 -> n_u and j = 1 -> n_v
            _subSurfs[i][j] = new GMlib::SubPatch<T>(_surface, _U_knotVector[i + 1], _V_knotVector[j + 1]);
        }

        if (_closedV){
            _subSurfs[i][_V_knotVector.size() - 3] = _subSurfs[i][0];
        }
    }


    if (_closedU){
      _subSurfs[_U_knotVector.size() - 3] = _subSurfs[0];
    }

}

template <typename T>
T BlendingSurface<T>::getIndex(T t, bool isU) const{
    std::vector<T> knotVector;
    T endValue;
    if(isU){
        knotVector = _U_knotVector;
        endValue = this->getEndPU();
    }
    else{
        knotVector = _V_knotVector;
        endValue = this->getEndPV();
    }

    if (t != endValue) {
      auto const next = std::upper_bound(knotVector.begin(), knotVector.end(), t);
      return std::distance(knotVector.begin(), next) - 1;
    }
    else {
      return knotVector.size() - 3;
    }
}

template <typename T>
T BlendingSurface<T>::W(T t, int i, int d, std::vector<T> const& knotVector) const{
    return (t - knotVector[i]) / (knotVector[i+d] - knotVector[i]);
}

template <typename T>
GMlib::Vector<T,2> BlendingSurface<T>::blend(T t) const{
    //{blend, derivided}
    return {3*t*t - 2*t*t*t, 6*t*(1-t)};
}
