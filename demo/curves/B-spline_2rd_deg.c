
//*****************************************
// Constructors and destructor           **
//*****************************************
template <typename T>
inline
BSpline<T>::BSpline( const GMlib::DVector<GMlib::Vector<T,3>>& c, bool blend, bool closed )
    : GMlib::PCurve<T,3>(0, 0, 0) , _c{c} {
    _closed = closed;
    _blend = blend;

    int n = _c.getDim();
    generateKnotVector(n, T(0), T(1));
}


template <typename T>
inline
BSpline<T>::BSpline( const GMlib::DVector<GMlib::Vector<T,3>>& p, int n, bool closed )
    : PCurve<T,3>(0, 0, 0) {
    _closed = closed;

    generateControlPoints(p , n);


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


    int index = getIndex(t);
    GMlib::Vector<T,3> b;


    if(_blend){
        b = Blend(t, index);
    }
    else{
      b = B(t, index);
    }


    auto pValue = b[0] * _c[index - 2];

    for( int i = 1 ; 2-i >= 0 ; i++){
        pValue += b[i] * _c[index - (2-i)];
    }


    this->_p[0] = pValue;
}


template <typename T>
T BSpline<T>::getStartP() const {
    return _knotVector[2];
}


template <typename T>
T BSpline<T>::getEndP()const {
    return _knotVector[_knotVector.size() - 3];
}


template <typename T>
void BSpline<T>::generateControlPoints( const GMlib::DVector<GMlib::Vector<T,3>>& p, int n ){
    auto xi = getXiVector(p);
    generateKnotVector(n, xi[0], xi.back());

    auto A_matrix = GMlib::DMatrix<T>(p.getDim(), n, T(0));

    for(int i = 0 ; i < xi.size() ; i++){
        int j = getIndex( xi[i] );

        auto b = B(xi[i], j);

        A_matrix[i][j - 2] = b[0];
        A_matrix[i][j - 1] = b[1];
        A_matrix[i][j] = b[2];
    }

    auto A_t = A_matrix;
    A_t.transpose();

    auto B_mat = A_t * A_matrix;
    auto y = A_t * p;

    B_mat.invert();

    _c = B_mat * y;
}


//Custom functions
template <typename T>
void BSpline<T>::generateKnotVector(int n, T start, T end){
    _knotVector.clear();

    auto dt = (end - start)/T(n - 2); //Distance between knots
    //auto dt = 1/T(n - 2); //Distance between knots

    // Total of 3 starts and 3 ends are added
    _knotVector.push_back(start);
    _knotVector.push_back(start);

    for (int i = start ; i <= (n-2) ; i++){
        _knotVector.push_back(start + (i * dt));
    }

    _knotVector.push_back(end);
    _knotVector.push_back(end);

//    for(auto pos : _knotVector){
//        std::cout << pos << std::endl;
//    }
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

    T w1 = (T(1) - wA) * (T(1) - wB);
    T w2 = (T(1) - wA) * wB + wA * (T(1) - wC);
    T w3 = wA * wC;

    return GMlib::Vector<T,3>(w1,w2,w3);
}


template <typename T>
GMlib::Vector<T,3> BSpline<T>::Blend(T t, int i) const {
    T bA = B_polynomial_function(W(t, 1, i)); //B(w_1,i(t))
    T bB = B_polynomial_function(W(t, 2, i-1)); //B(w_2,i-1)
    T bC = B_polynomial_function(W(t, 2, i)); //B(w_2,i)

    T b1 = (1 - bA) * (1 - bB);
    T b2 = (1 - bA) * bB + bA * (1 - bC);
    T b3 = bA * bC;

    return GMlib::Vector<T,3>(b1,b2,b3);
}

template <typename T>
T BSpline<T>::B_polynomial_function(T t) const {
    return 3*t*t - 2*t*t*t;
}


template <typename T>
int BSpline<T>::getIndex(T t) const {
    if (t >= this->getEndP()){
        return _knotVector.size() - 4;
    }
    else{
        return std::distance(_knotVector.begin(), std::upper_bound(_knotVector.begin(), _knotVector.end(), t)) -1;
    }
}

template <typename T>
std::vector<T> BSpline<T>::getXiVector(const GMlib::DVector<GMlib::Vector<T,3>>& p){
    auto x_vec = std::vector<T>{T(0)};

    for (int i = 1 ; i < p.getDim(); i++) {
      x_vec.push_back((p[i] - p[i - 1]).getLength() + x_vec.back());
    }

    return x_vec;
}
