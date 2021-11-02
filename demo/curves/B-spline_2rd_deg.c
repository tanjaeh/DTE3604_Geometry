
//*****************************************
// Constructors and destructor           **
//*****************************************
template <typename T>
inline
BSpline<T>::BSpline( const GMlib::DVector<GMlib::Vector<T,3>>& c, bool closed )
    : GMlib::PCurve<T,3>(0, 0, 0) , _c{c} {
    _closed = closed;

    int n = _c.getDim();
    generateKnotVector(n);
}


template <typename T>
inline
BSpline<T>::BSpline( const GMlib::DVector<GMlib::Vector<T,3>>& p, int n, bool closed ) : PCurve<T,3>(0, 0, 0) {


}


template <typename T>
inline
BSpline<T>::BSpline( const BSpline<T>& copy ) : PCurve<T,3>(copy) {}


//The destructor clean up and destroy all private data
template <typename T>
BSpline<T>::~BSpline() {}



  //**************************************
  //        Public local functons       **
  //**************************************



  //***************************************************
  // Overrided (public) virtual functons from PCurve **
  //***************************************************
template <typename T>
bool BSpline<T>::isClosed() const {
    return _closed;
}



  //******************************************************
  // Overrided (protected) virtual functons from PCurve **
  //******************************************************
template <typename T>
void BSpline<T>::eval( T t, int d, bool l ) const { //bool l tells if it eval from left (default True)
    //Next line must join all eval functions
    this->_p.setDim( d + 1 );


    //int i = getIndex(t);
    int i;
    if (t == this->getEndP()){
        i = _knotVector.size() - 4;
    } else{
        i = std::distance(_knotVector.begin(), std::upper_bound(_knotVector.begin(), _knotVector.end(), t)) -1;
    }

    auto b = B(t, i);

    this->_p[0] =
            b[0] * _c[i-2] +
            b[1] * _c[i-1] +
            b[2] * _c[i];

}


template <typename T>
T BSpline<T>::getStartP() const {
    return T(0);
}


template <typename T>
T BSpline<T>::getEndP()const {
    return T(1);
}


//template <typename T>
//void BSpline<T>::computeSurroundingSphere( const std::vector<GMlib::DVector<GMlib::Vector<T,3>>>& /*p*/, GMlib::Sphere<T,3>& s ) const {

//}

template <typename T>
void BSpline<T>::generateKnotVector(int n){
    _knotVector.clear();

    auto start = T(0);
    auto end = T(1);

    auto dt = (end - start)/T(n - 2); //Distance between knots

    // Total of 3 starts and 3 ends are added
    _knotVector.push_back(start);
    _knotVector.push_back(start);

    for (auto i = start ; i <= (n-2) ; i++){
        _knotVector.push_back(i*dt);
    }

    _knotVector.push_back(end);
    _knotVector.push_back(end);
}


template <typename T>
T BSpline<T>::W(T t, int d, int i) const {
    return (t - _knotVector[i]) / (_knotVector[i+d] - _knotVector[i]);
}


template <typename T>
GMlib::Vector<T,3> BSpline<T>::B(T t, int i) const {
    T wA = W(t, 1, i); //w_1,i(t)
    T wB = W(t, 2, i-1); //w_2,i-1
    T wC = W(t, 2, i); //w_2,i

    T w1 = (1 - wA) * (1 - wB);
    T w2 = (1 - wA) * wB + wA * (1 - wB);
    T w3 = wA * wC;

    return GMlib::Vector<T,3>(w1,w2,w3);
}


template <typename T>
int BSpline<T>::getIndex(T t) {
    int index = std::upper_bound(_knotVector.begin(), _knotVector.end(), t);
    return index;
}
