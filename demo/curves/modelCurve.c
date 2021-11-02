
//*****************************************
// Constructors and destructor           **
//*****************************************
template <typename T>
inline
ModelCurve<T>::ModelCurve( T radius ) : GMlib::PCurve<T,3>(20, 0, 7) {
    //Default default radius = 20
    // Note that the last parameter in the PCurve constructor is 7,
    // this because 7 derivatives in eval() is implemented!!!!
    _rx = radius;
    _ry = radius;
}


template <typename T>
inline
ModelCurve<T>::ModelCurve(  T radius1, T radius2 ) : PCurve<T,3>(20, 0, 7) {
    //Default default radius = 20
    // Note that the last parameter in the PCurve constructor is 7,
    // this because 7 derivatives in eval() is implemented!!!!
    _rx = radius1;
    _ry = radius2;
}


template <typename T>
inline
ModelCurve<T>::ModelCurve( const ModelCurve<T>& copy ) : PCurve<T,3>(copy) {}


//The destructor clean up and destroy all private data
template <typename T>
ModelCurve<T>::~ModelCurve() {}



  //**************************************
  //        Public local functons       **
  //**************************************
template <typename T>
inline
T ModelCurve<T>::getRadius(int i) const {
    if(i==1)
        return _rx;
    else {
        return _ry;
    }
}


template <typename T>
inline
void ModelCurve<T>::setRadius( T radius ) {
    _rx = _ry = radius;
}


template <typename T>
inline
void ModelCurve<T>::setRadius( T radius1, T radius2 ) {
    _rx = radius1;
    _ry = radius2;
}



  //***************************************************
  // Overrided (public) virtual functons from PCurve **
  //***************************************************
template <typename T>
bool ModelCurve<T>::isClosed() const {
    return true;
}



  //******************************************************
  // Overrided (protected) virtual functons from PCurve **
  //******************************************************
template <typename T>
void ModelCurve<T>::eval( T t, int d, bool /*l*/ ) const { //bool l tells if it eval from left (default True)
    //Next line must join all eval functions
    this->_p.setDim( d + 1 );


    auto b_x = _rx/4;
    auto b_y = _ry/4;
    const T ct_x = 3 * b_x * cos(t) + b_x * cos(3*t);
    const T st_y = 3 * b_y * sin(t) - b_y * sin(3*t);

    this->_p[0][0] = ct_x;
    this->_p[0][1] = st_y;
    this->_p[0][2] = T(0);

    if( this->_dm == GMlib::GM_DERIVATION_EXPLICIT ) {
      const T st_x = _rx * sin(t);
      const T ct_y = _ry * cos(t);
      if( d > 0 ) {
        this->_p[1][0] = -st_x;
        this->_p[1][1] =  ct_y;
        this->_p[1][2] =  T(0);
      }
      if( d > 1 ) this->_p[2] = -this->_p[0];
      if( d > 2 ) this->_p[3] = -this->_p[1];
      if( d > 3 ) this->_p[4] =  this->_p[0];
      if( d > 4 ) this->_p[5] =  this->_p[1];
      if( d > 5 ) this->_p[6] =  this->_p[2];
      if( d > 6 ) this->_p[7] =  this->_p[3];
    }
}


template <typename T>
T ModelCurve<T>::getStartP() const {
    return T(0);
}


template <typename T>
T ModelCurve<T>::getEndP()const {
    return T( M_2PI );
}


template <typename T>
void ModelCurve<T>::computeSurroundingSphere( const std::vector<GMlib::DVector<GMlib::Vector<T,3>>>& /*p*/, GMlib::Sphere<T,3>& s ) const {

    s.resetPos(GMlib::Point<T,3>(T(0)));
    if(_rx > _ry)
        s.resetRadius(_rx);
    else
        s.resetRadius(_ry);
}

