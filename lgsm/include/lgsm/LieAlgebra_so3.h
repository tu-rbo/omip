// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_LIE_ALGEBRA_so3_H
#define EIGEN_LGSM_LIE_ALGEBRA_so3_H

/***************************************************************************
* Function used in exp derivatives
***************************************************************************/

template<typename Scalar> inline Scalar ei_lie_algebra_so3_derivative_f2(Scalar n, Scalar n2, Scalar cos_n, Scalar precision){
  if(n2 < precision)
    return  (Scalar(0.5) + (Scalar(-1.) + n2 / Scalar(30.)) * n2 / Scalar(24.) );
  else
    return (Scalar(1.0) - cos_n)/n2;
}

template<typename Scalar> EIGEN_STRONG_INLINE Scalar ei_lie_algebra_so3_derivative_df2(Scalar n, Scalar n2, Scalar cos_n, Scalar sin_n, Scalar precision){ // 1.e-2
  if ( n2 < precision )
    return ( (Scalar(-1.)+(Scalar(1.)/Scalar(15.)+(Scalar(-1.)/Scalar(560.)+n2/Scalar(37800.))*n2)*n2)/Scalar(12.) );
  else
    return ( Scalar(2.)*( Scalar(0.5)*(n2-Scalar(2.)+Scalar(2.)*cos_n )/(n2*n2) ) - (n - std::sin(n))/(n2*n) );
}

template<typename Scalar> inline Scalar ei_lie_algebra_so3_derivative_f3(Scalar n, Scalar n2, Scalar sin_n, Scalar precision){
  if(n2 < precision)
    return (Scalar(20.)+(Scalar(-1.)+n2/Scalar(42.))*n2)/Scalar(120.);
  else
    return (n - sin_n)/(n2*n);
}

template<typename Scalar> EIGEN_STRONG_INLINE Scalar ei_lie_algebra_so3_derivative_df3(Scalar n, Scalar n2, Scalar cos_n, Scalar sin_n, Scalar precision){ // 1.e-2
  if (n2 < precision)
    return (-Scalar(21.)+(Scalar(1.)+(-Scalar(1.)/Scalar(48.)+Scalar(1.)/Scalar(3960.)*n2)*n2)*n2)/Scalar(1260.) ;
  else
    return Scalar(3)*( (n*n2-Scalar(6)*n+Scalar(6)*sin_n)/(Scalar(6)*n*n2*n2) ) - ( Scalar(0.5)*(n2-Scalar(2)+Scalar(2)*cos_n )/(n2*n2) );
}

/***********************************************************************************
* Definition/implementation of LieAlgebraBase<Matrix<Scalar, 3, 1>, Derived>
************************************************************************************/

/**
 * \brief Base class for the Lie Algebra so(3). 
 *
 * \tparam Derived the derived class holding the coefficients which are of type Array<Scalar, 3, 1> or Map<Array<Scalar, 3, 1> >
 *
 * This class actually implements methods form LieAlgebraBase for so(3)
 *
 * Since a Lie Algebra is a vector Space (check if it's true in the general case) it's inherited from MatrixBase
 */

template<class Derived> 
class LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 3, 1>, Derived> : 
  public MatrixBase<Derived> 
{
protected:
  /** The inherited class */
  typedef MatrixBase<Derived> Base;
public:  
  // inherit MatrixBase interface
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebraBase)
  // inherit operator= [XXX] inheriting Base::opertor= through warning C4522 with visual studio : multiple assignment operators specified 
  //EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebraBase)
  // accessor needed for MatrixBase inheritance
  LIE_INHERIT_MATRIX_BASE(3, 1)

  /** The wrapped class */
  typedef Matrix<Scalar, 3, 1> BaseType;
  /** The kind of stored coefficients */
  typedef typename internal::traits<Derived>::Coefficients Coefficients;
  /** The plain object returned, while using Map<LieAlgebra< > >*/
  typedef LieAlgebra<BaseType> PlainObject;   
  /** The type of the dual Algebra */
  typedef LieAlgebraDual<BaseType> AlgebraDual;
  /** The type of the associated Lie Group */
  typedef typename internal::traits<Derived>::Group Group;
  /** The type of the matrix return by derivatives of exp */
  typedef Matrix<Scalar, 3, 3> Matrix3;

  /** Default assignement operator */
  EIGEN_STRONG_INLINE LieAlgebraBase& operator=(const LieAlgebraBase& other);
  /** Assignement operator between derived type*/
  template<class OtherDerived> EIGEN_STRONG_INLINE Derived& operator=(const LieAlgebraBase<BaseType, OtherDerived>& other);

  template<class OtherDerived> EIGEN_STRONG_INLINE Derived& operator=(const MatrixBase<OtherDerived>& other) {
    this->get() = other;
    return derived();
  }


  /** Lie Bracket*/
  template<class OtherDerived> inline PlainObject bracket(const LieAlgebraBase<BaseType, OtherDerived>& a) const;
//  template<class OtherDerived> inline PlainObject adjoint(const LieAlgebraBase<BaseType, OtherDerived>& a) const;

  /** \returns The element of the associated Lie Group through the exponential map
   */
  inline Group exp(Scalar precision = 1.e-5) const;
  /** \return the first derivative of exp
    * \sa LieAlgebraBase::exp
    */
  inline Matrix<Scalar, 3, 3> dexp(Scalar precision = 1.e-6, Scalar precision2 = 1.e-2) const;
  /** \returns second derivative of exp along vector \c v 
    * \sa LieAlgebraBase::exp
    */
  template<class OtherDerived> Matrix<Scalar, 3, 3> inline d2exp(const MatrixBase<OtherDerived>& v, Scalar precision = 1.e-6, Scalar precision2 = 1.e-2) const;
  
  /** \returns The stored coefficients */
  inline Coefficients& get() {return this->derived().get(); }
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const {return this->derived().get(); }
};

/***************************************************************
 * Implementation of LieAlgebraBase<Array<Scalar, 3, 1> > methods
 ***************************************************************/

// assignation 

template<class Derived>
inline LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 3, 1>, Derived>& 
  LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 3, 1>, Derived>::operator=(const LieAlgebraBase& other) 
{
  this->get() = other.get();
  return *this;
}

template<class Derived>
template<class OtherDerived>
inline Derived& 
  LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 3, 1>, Derived>::operator=(const LieAlgebraBase<BaseType, OtherDerived>& other) 
{
  this->get() = other.get();
  return derived();
}

// bracket
template<class Derived> 
template<class OtherDerived>
inline typename LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 3, 1>, Derived>::PlainObject 
  LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 3, 1>, Derived>::bracket(const LieAlgebraBase<Matrix<Scalar, 3, 1>, OtherDerived>& ang) const 
{
  return this->cross(ang);
}

// adjoint
/*template<class Derived, typename _Scalar> 
template<class OtherDerived>
typename LieAlgebraBase<Matrix<_Scalar, 3, 1>, Derived >::PlainObject LieAlgebraBase<Matrix<_Scalar, 3, 1>, Derived >::adjoint(const LieAlgebraBase<Matrix<Scalar, 3, 1>, OtherDerived>& a) const {
  return this->bracket(ang);
}*/

// exp
template<class Derived> 
inline typename LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 3, 1>, Derived >::Group 
 LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar,3, 1>, Derived >::exp(const Scalar precision) const 
{
  const Scalar n2 = this->squaredNorm();

  if (n2 < precision)
  {
    const Scalar w = Scalar(1.0) + (Scalar(-1.0) + n2 / Scalar(48.0)) * n2 / Scalar(8.0); // Series expansion of cos(n/2) O(4)
    return Group(w, (Scalar(1.0) + (Scalar(-1.0) + Scalar(0.0125) * n2) * n2 / Scalar(24.0)) / Scalar(2.0) * *this); // Series expansion of sin(n/2)/n O(4)
  }
  else
  {
    const Scalar n = std::sqrt(n2);
    const Scalar w = std::cos(n * Scalar(0.5));
    return Group(w, std::sin(n * Scalar(0.5)) / n * *this);
  }
}

//dexp
template<class Derived> 
inline typename LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 3, 1>, Derived >::Matrix3  
 LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 3, 1>, Derived >::dexp(Scalar precision, 
                                                                                                 Scalar precision2) const 
{
  Scalar n2 = this->squaredNorm();
  Scalar n = std::sqrt(n2);
  Scalar sin_n = std::sin(n);
  Scalar cos_n = std::cos(n);

  Scalar f2 = ei_lie_algebra_so3_derivative_f2(n, n2, cos_n, precision);
  Scalar f3 = ei_lie_algebra_so3_derivative_f3(n, n2, sin_n, precision2);

  Matrix<Scalar, 3, 3> m;

  m(0,0) = Scalar(1.0) - f3 * (n2 - this->coeff(0)*this->coeff(0));
  m(1,1) = Scalar(1.0) - f3 * (n2 - this->coeff(1)*this->coeff(1));
  m(2,2) = Scalar(1.0) - f3 * (n2 - this->coeff(2)*this->coeff(2));

  m(1, 0) = f3 * this->coeff(0) * this->coeff(1) - f2 * this->coeff(2);
  m(0, 1) = m(1,0) + 2 * f2 * this->coeff(2);

  m(2, 0) = f3 * this->coeff(0) * this->coeff(2) + f2 * this->coeff(1);
  m(0, 2) = m(2,0) - 2 * f2 * this->coeff(1);

  m(2, 1) = f3 * this->coeff(1) * this->coeff(2) - f2 * this->coeff(0);
  m(1, 2) = m(2,1) + 2 * f2 * this->coeff(0);

  return m;
}

//d2exp
template<class Derived> 
template<class OtherDerived> 
inline typename LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 3, 1>, Derived >::Matrix3 
 LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar,3, 1>, Derived >::d2exp(const MatrixBase<OtherDerived>& vec, 
                                                                                                 Scalar precision, 
                                                                                                 Scalar precision2) const
{
  Scalar n2 = this->squaredNorm();
  Scalar n = std::sqrt(n2);
  Scalar cos_n = std::cos(n);
  Scalar sin_n = std::sin(n);

  Scalar f2 = ei_lie_algebra_so3_derivative_f2(n, n2, cos_n, precision);
  Scalar f3 = ei_lie_algebra_so3_derivative_f3(n, n2, sin_n, precision2);
  Scalar df2 = ei_lie_algebra_so3_derivative_df2(n, n2, cos_n, sin_n, precision2);
  Scalar df3 = ei_lie_algebra_so3_derivative_df3(n, n2, cos_n, sin_n, precision2);

  Scalar vw = this->dot(vec);

  // computing w*v^T may be faster

  Matrix<Scalar, 3, 3> m;

  m(0,0) = - 2 * f3 * (this->coeff(2) * vec[2] + this->coeff(1) * vec[1]) - df3 * vw * (n2 - this->coeff(0)*this->coeff(0));
  m(1,1) = - 2 * f3 * (this->coeff(2) * vec[2] + this->coeff(0) * vec[0]) - df3 * vw * (n2 - this->coeff(1)*this->coeff(1));
  m(2,2) = - 2 * f3 * (this->coeff(1) * vec[1] + this->coeff(0) * vec[0]) - df3 * vw * (n2 - this->coeff(2)*this->coeff(2));

  m(1, 0) = -vec[2] * f2 +  f3 * (this->coeff(0)*vec[1] + this->coeff(1)*vec[0]) + vw * (df3 * this->coeff(0) * this->coeff(1) - df2 * this->coeff(2));
  m(0, 1) = m(1,0) + 2 * vec[2] * f2 + 2 * vw * df2 * this->coeff(2);

  m(2, 0) = vec[1] * f2  +  f3 * (this->coeff(0)*vec[2] + this->coeff(2)*vec[0]) + vw * (df3 * this->coeff(0) * this->coeff(2) + df2 * this->coeff(1));
  m(0, 2) = m(2,0) - 2 * vec[1] * f2 - 2 * vw * df2 * this->coeff(1);

  m(2, 1) = -vec[0] * f2 +  f3 * (this->coeff(2)*vec[1] + this->coeff(1)*vec[2]) + vw * (df3 * this->coeff(1) * this->coeff(2) - df2 * this->coeff(0));
  m(1, 2) = m(2,1) + 2 * vec[0] * f2 + 2 * vw * df2 * this->coeff(0);

  return m;
}


/***********************************************************************************
* Definition/implementation of LieAlgebraDualBase<Matrix<Scalar, 3, 1>, Derived>
************************************************************************************/

namespace internal {
  template<typename Scalar>
    struct traits<LieAlgebraDual<Matrix<Scalar, 3, 1> > > : public traits<Matrix<Scalar, 3, 1> >
    {
      typedef Matrix<Scalar, 3, 1> Coefficients;
      typedef LieGroup<Quaternion<Scalar> > Group;
    };

  template<typename Scalar, int MapOptions>
    struct traits<Map<LieAlgebraDual<Matrix<Scalar, 3, 1> >, MapOptions> > : public traits<Map<Matrix<Scalar, 3, 1>, MapOptions> >
    {
      typedef Map<Matrix<Scalar, 3, 1>, MapOptions> Coefficients;
      typedef LieGroup<Quaternion<Scalar> > Group;
    };

  template<typename Scalar, int MapOptions>
    struct traits<Map<const LieAlgebraDual<Matrix<Scalar, 3, 1> >, MapOptions> > : public traits<Map<const Matrix<Scalar, 3, 1>, MapOptions> >
    {
      typedef Map<const Matrix<Scalar, 3, 1>, MapOptions> Coefficients;
      typedef LieGroup<Quaternion<Scalar> > Group;
    };

}

// no function to implement

/***************************************************************************
 * Definition/implementation of LieAlgebra<Matrix<Scalar, 3, 1> >
 ***************************************************************************/

/**
 * \brief Class for the so(3) Lie Algebra.  
 *
 * \tparam _Scalar the type of the underlying array
 *
 * This class is a specialization of LieAlgebra. It adds specific constructor for so(3). 
 *
 * \sa The methods are defined in LieAlgebraBase
 */

namespace internal {
  template<typename Scalar>
    struct traits<LieAlgebra<Matrix<Scalar, 3, 1> > > 
    : public traits<LieAlgebraBase<Matrix<Scalar, 3, 1>, LieAlgebra<Matrix<Scalar, 3, 1> > > >
    {
      typedef Matrix<Scalar, 3, 1> Coefficients;
      typedef LieGroup<Quaternion<Scalar> > Group;
    };

  template<typename Scalar, int Options>
    struct traits<Map<LieAlgebra<Matrix<Scalar, 3, 1> >, Options> > 
    : public traits<LieAlgebraBase<Matrix<Scalar, 3, 1>, LieAlgebra<Matrix<Scalar, 3, 1> > > >
    {
      typedef Map<Matrix<Scalar, 3, 1>, Options> Coefficients;
      typedef LieGroup<Quaternion<Scalar> > Group;
    };

  template<typename Scalar, int Options>
    struct traits<Map<const LieAlgebra<Matrix<Scalar, 3, 1> >, Options> > 
    : public traits<LieAlgebraBase<Matrix<Scalar, 3, 1>, LieAlgebra<Matrix<Scalar, 3, 1> > > >
    {
      typedef Map<const Matrix<Scalar, 3, 1>, Options> Coefficients;
      typedef LieGroup<Quaternion<Scalar> > Group;
    };

}

template<typename _Scalar> 
class LieAlgebra<Matrix<_Scalar, 3, 1> > : 
  public LieAlgebraBase<Matrix<_Scalar, 3, 1>, LieAlgebra<Matrix<_Scalar, 3, 1> > > 
{
protected:
  /** The inherited class */
  typedef LieAlgebraBase<Matrix<_Scalar, 3, 1>, LieAlgebra<Matrix<_Scalar, 3, 1> > > Base;
public:
  // inherit MatrixBase operator
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebra)
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebra)

  /** The stored coefficients */
  typedef typename internal::traits<LieAlgebra<Matrix<Scalar, 3, 1> > >::Coefficients Coefficients;

  /** Default constructor */
  inline LieAlgebra() : m_coeffs() {}
  /** Copy constructor */
  inline LieAlgebra(const LieAlgebra& a) : m_coeffs(a.get() ) {} // maybe supernumerary
  /** Copy constructor */
  template<class Derived>
  inline LieAlgebra(const LieAlgebraBase<typename Base::BaseType, Derived>& a) : m_coeffs(a.get() ) {}
  /** Copy constructor */
  inline LieAlgebra(const Coefficients& a) : m_coeffs(a) {}
  /** Constructs an element of so(3) from 3 scalar \c x \c y \c z*/
  inline LieAlgebra(Scalar x, Scalar y, Scalar z) : m_coeffs(x, y, z) {}

  /** \returns The stored coefficients */
  inline Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const { return m_coeffs; }

protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};


/*****************************************************************************************
* Definition/implementation of Map<LieAlgebra<Matrix<Scalar, 3, 1>  > >
******************************************************************************************/

// no new constructor needed

/*****************************************************************************************
* Definition/implementation of LieAlgebraDual<Matrix<Scalar, 3, 1>  >
******************************************************************************************/

template<typename _Scalar> 
class LieAlgebraDual<Matrix<_Scalar, 3, 1> > : 
  public LieAlgebraDualBase<Matrix<_Scalar, 3, 1>, LieAlgebraDual<Matrix<_Scalar, 3, 1> > > 
{
protected:
  /** The inherited class */
  typedef LieAlgebraDualBase<Matrix<_Scalar, 3, 1>, LieAlgebraDual<Matrix<_Scalar, 3, 1> > > Base;
public:
  // inherit MatrixBase operator
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebraDual)
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebraDual)

  /** The stored coefficients */
  typedef typename internal::traits<LieAlgebraDual<Matrix<Scalar, 3, 1> > >::Coefficients Coefficients;

  /** Default constructor */
  inline LieAlgebraDual() : m_coeffs() {}
  /** Copy constructor */
  inline LieAlgebraDual(const LieAlgebraDual& a) : m_coeffs(a.get() ) {}
  /** Copy constructor */
  inline LieAlgebraDual(const Coefficients& a) : m_coeffs(a) {}
  /** Constructs an element of so(3) from 3 scalar \c x \c y \c z*/
  inline LieAlgebraDual(Scalar x, Scalar y, Scalar z) : m_coeffs(x, y, z) {}

  /** \returns The stored coefficients */
  inline Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const { return m_coeffs; }

protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};

/*****************************************************************************************
* Definition/implementation of Map<LieAlgebraDual<Matrix<Scalar, 3, 1>  > >
******************************************************************************************/

// no new constructor needed

#endif
