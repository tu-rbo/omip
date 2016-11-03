// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_LIE_ALGEBRA_se3_H
#define EIGEN_LGSM_LIE_ALGEBRA_se3_H

/***********************************************************************************
* Definition/implementation of LieAlgebraBase<Matrix<Scalar, 6, 1>, Derived>
************************************************************************************/

/**
 * \brief Base class for the Lie Algebra se(3). 
 *
 * \tparam Derived the derived class holding the coefficients which are of type Array<Scalar, 6, 1> or Map<Array<Scalar, 6, 1> >
 *
 * This class actually implements methods form LieAlgebraBase for se(3). Since se(3) is the semi direct product of R^3 
 * and so(3) many operations are performed using directly elements form R^3 or so(3)
 *
 * a Lie Algebra is also a vector Space (check if it's true in the general case) that's why it's inherited from MatrixBase
 */

template<class Derived> 
class LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived> 
  : public MatrixBase<Derived> 
{
protected:
  /** The inherited class */
  typedef MatrixBase<Derived> Base;
public:  
  // inherit MatrixBase interface
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebraBase)
  // inherit operator= [XXX] inheriting Base::opertor= through warning C4522 with visual studio : multiple assignment operators specified 
  // EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebraBase)
  // accessor needed for MatrixBase inheritance
  LIE_INHERIT_MATRIX_BASE(6, 1)

  /** The wrapped class */
  typedef Matrix<Scalar, 6, 1> BaseType;
  /** The kind of stored coefficients */
  typedef typename internal::traits<Derived>::Coefficients Coefficients;
  /** The plain object returned, while using Map<LieAlgebra< > >*/
  typedef LieAlgebra<BaseType> PlainObject;   
  /** The type of the dual Algebra */
  typedef LieAlgebraDual<BaseType> AlgebraDual;
  /** The type of the associated Lie Group */
  typedef typename internal::traits<Derived>::Group Group;
  /** The type of the matrix return by derivatives of exp */
  typedef Matrix<Scalar, 6, 6> Matrix6;

  /** The type of an element of R^3*/ 
  typedef Matrix<Scalar, 3, 1> Vector3;     
  /** The type of an element of so(3) */             
  typedef LieAlgebra<Matrix<Scalar, 3, 1> > so3Element;


  /** Default assignement operator */
  EIGEN_STRONG_INLINE LieAlgebraBase& operator=(const LieAlgebraBase& other);
  /** Assignement operator between derived type*/
  template<class OtherDerived> EIGEN_STRONG_INLINE Derived& operator=(const MatrixBase<OtherDerived>& other);
  
  /** Lie Bracket*/
  template<class OtherDerived> PlainObject bracket(const LieAlgebraBase<BaseType, OtherDerived>& a) const;
  /** \returns The element of the associated Lie Group through the exponential map
   */
  inline Group exp(const Scalar precision = 1.e-6) const;
  /** \return the first derivative of exp
    * \sa LieAlgebraBase::exp
    */
  inline Matrix<Scalar, 6, 6> dexp() const; 
  /*
  inline Map<so3Element> getso3Element(){ return Map<so3Element>(this->get()); }
  inline const Map<so3Element> getso3Element() const {return  Map<so3Element>(this->get()); }
  inline VectorBlock<Derived, 3> getR3Element(){ return this->tail<3>(); }
  inline const VectorBlock<Derived, 3> getR3Element() const { return this->tail<3>(); }*/

  /** The accessor to the so(3) element */
  inline Map<so3Element> getso3Element(){ return Map<so3Element>(this->derived().get().template head<3>().data()); }
  /** The read-only accessor to the so(3) element */
  inline Map<const so3Element> getso3Element() const {return  Map<const so3Element>(this->derived().get().template head<3>().data()); }
  /** The accessor to the R^3 element */
  inline Map<Vector3> getR3Element() { return  Map<Vector3>(this->derived().get().template tail<3>().data()); }
  /** The read-only accessor to the R^3 element */
  inline Map<const Vector3> getR3Element() const { return  Map<const Vector3>(this->derived().get().template tail<3>().data()); }

  /** \returns The stored coefficients by the derived class*/
  Coefficients& get() { return derived().get(); }
  /** \returns The read-only access to the stored coefficients by the derived class*/
  const Coefficients& get() const { return derived().get(); }
};

/***************************************************************
 * Implementation of LieAlgebraBase<Array<Scalar, 6, 1> > methods
 ***************************************************************/

// assignation
template<class Derived>
inline LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived>& 
  LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived>::operator=(const LieAlgebraBase& other) 
{
  this->get() = other.get();
  return *this;
}

template<class Derived>
template<class OtherDerived>
inline Derived& 
  LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived>::operator=(const MatrixBase<OtherDerived>& other) 
{
  this->get() = other;
  return derived();
}


// bracket 
template<class Derived> 
template<class OtherDerived>
  typename LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived>::PlainObject 
  LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived>::bracket(const LieAlgebraBase<Matrix<Scalar, 6, 1>, 
                                                                                                    OtherDerived>& a) const 
{
  return PlainObject(this->getso3Element().cross(a.getso3Element()), 
                     this->getso3Element().cross(a.getR3Element()) - a.getso3Element().cross(this->getR3Element()) );
}
/*
// adjoint
template<class Derived> 
template<class OtherDerived>
  typename LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived >::PlainObject 
  LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived >::adjoint(const LieAlgebraBase<Matrix<Scalar, 6, 1>, 
                                                                                                     OtherDerived>& a) const 
{
  return this->bracket(ang);
}*/

// exp -> unrolling this expression could be 30% faster.
template<class Derived> inline
typename LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived>::Group 
  LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived>::exp(const Scalar precision) const 
{
  return Group(this->getso3Element().dexp().transpose() * this->getR3Element()
               ,this->getso3Element().exp());
}

// dexp -> unrolling this expression maybe faster
template<class Derived>
inline typename LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived >::Matrix6  
  LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived>::dexp() const 
{ 
  Matrix<Scalar, 6, 6> res;

  // gcc does not seem to like res.block<3,3>(0,0) = this->getso3Element().dexp();
  res.template block<3,3>(0,0) = this->getso3Element().dexp();
  res.template block<3,3>(3,3) = res.template block<3,3>(0,0);
  res.template block<3,3>(0,3).setZero();
  res.template block<3,3>(3,0) = this->getso3Element().d2exp(this->getR3Element());

  return res;
}


/*************************************************************************************
* Definition/implementation of LieAlgebraDualBase<Matrix<Scalar, 6, 1>, Derived>
**************************************************************************************/

/**
 * \brief Base class for the Lie algebra dual se*(3). 
 *
 * \tparam Derived the derived class holding the coefficients which are of type Array<Scalar, 6, 1> or Map<Array<Scalar, 6, 1> >
 *
 * This class actually implements methods form LieAlgebraDualBase for se*(3). Since se*(3) is the semi direct product of R^3 
 * and so*(3) many operations are performed using directly elements form R^3 or so*(3)
 *
 * a Lie algebra dual is also a vector Space (check if it's true in the general case) that's why it's inherited from MatrixBase
 */

template<class Derived> 
class LieAlgebraDualBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived> : public MatrixBase<Derived> {
protected:
  /** The inherited class */
  typedef MatrixBase<Derived>  Base;
public:
  // inherit MatrixBase interface
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebraDualBase)
  // inherit operator= [XXX] inheriting Base::opertor= through warning C4522 with visual studio : multiple assignment operators specified 
  // EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebraDualBase)
  // accessor needed for MatrixBase inheritance
  LIE_INHERIT_MATRIX_BASE(6, 1)

  /** The wrapped class */
  typedef Matrix<Scalar, 6, 1> BaseType;
  /** The kind of stored coefficients */
  typedef typename internal::traits<Derived>::Coefficients Coefficients;
  /** The plain object returned, while using Map<LieAlgebra< > >*/
  typedef LieAlgebraDual<BaseType> PlainObject;   
  /** The type of the dual Algebra */
  typedef LieAlgebra<BaseType> Algebra;

  /** The type of an element of R^3*/ 
  typedef Matrix<Scalar, 3, 1> Vector3;     
  /** The type of an element of so*(3) */ 
  typedef LieAlgebraDual<Matrix<Scalar, 3, 1> > so3Element;
  
  /** The accessor to the so(3) element */
  inline Map<so3Element> getso3Element(){ return Map<so3Element>(this->derived().get().template head<3>().data()); }
  /** The read-only accessor to the so(3) element */
  inline Map<const so3Element> getso3Element() const {return  Map<const so3Element>(this->derived().get().template head<3>().data()); }
  /** The accessor to the R^3 element */
  inline Map<Vector3> getR3Element() { return  Map<Vector3>(this->derived().get().template tail<3>().data()); }
  /** The read-only accessor to the R^3 element */
  inline Map<const Vector3> getR3Element() const { return  Map<const Vector3>(this->derived().get().template tail<3>().data()); }
  
  /** \returns The stored coefficients by the derived class*/
  Coefficients& get() { return derived().get(); }
  /** \returns The read-only access to the stored coefficients by the derived class*/
  const Coefficients& get() const { return derived().get(); }
};

/***************************************************************************
* Definition/implementation of LieAlgebra<Matrix<Scalar, 6, 1> >
***************************************************************************/

/**
 * \brief Class for the se(3) Lie Algebra.  
 *
 * \tparam _Scalar the type of the underlying array
 *
 * This class is a specialization of LieAlgebra. It adds specific constructor for se(3). 
 *
 * \sa The methods are defined in LieAlgebraBase
 */

namespace internal {
  template<typename Scalar>
    struct traits<LieAlgebra<Matrix<Scalar, 6, 1> > > 
    : public traits<LieAlgebraBase<Matrix<Scalar, 6, 1>, LieAlgebra<Matrix<Scalar, 6, 1> > > >
    {
      typedef Matrix<Scalar, 6, 1> Coefficients;
      typedef LieGroup<Array<Scalar, 7, 1> > Group;
    };

  template<typename Scalar, int Options>
    struct traits<Map<LieAlgebra<Matrix<Scalar, 6, 1> >, Options> > 
    : public traits<LieAlgebraDualBase<Matrix<Scalar, 6, 1>, LieAlgebraDual<Matrix<Scalar, 6, 1> > > >
    {
      typedef Map<Matrix<Scalar, 6, 1>, Options> Coefficients;
      typedef LieGroup<Array<Scalar, 7, 1> > Group;
    };

  template<typename Scalar, int Options>
    struct traits<Map<const LieAlgebra<Matrix<Scalar, 6, 1> >, Options> > 
    : public traits<LieAlgebraDualBase<Matrix<Scalar, 6, 1>, LieAlgebraDual<Matrix<Scalar, 6, 1> > > >
    {
      typedef Map<const Matrix<Scalar, 6, 1>, Options> Coefficients;
      typedef LieGroup<Array<Scalar, 7, 1> > Group;
    };

}

template<typename _Scalar> class LieAlgebra<Matrix<_Scalar, 6, 1> > : 
  public LieAlgebraBase<Matrix<_Scalar, 6, 1>, LieAlgebra<Matrix<_Scalar, 6, 1> > > 
{
protected:
  /** The inherited class */
  typedef LieAlgebraBase<Matrix<_Scalar, 6, 1>, LieAlgebra<Matrix<_Scalar, 6, 1> > > Base;
public:
  // inherit MatrixBase operator
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebra)
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebra)

  /** The type of the underlying array */
  typedef typename Base::BaseType BaseType;
  /** The stored coefficients */
  typedef typename internal::traits<LieAlgebra<Matrix<Scalar, 6, 1> > >::Coefficients Coefficients;

  /** Default constructor */
  inline LieAlgebra() : m_coeffs() {}
  /** Copy constructor */
  inline LieAlgebra(const LieAlgebra& g) : m_coeffs(g.get() ) {}
  /** Copy constructor */
  inline LieAlgebra(const BaseType& g) : m_coeffs(g) {}
  /** Constructs an element of se(3) from 6 scalar */
  inline LieAlgebra(Scalar rx, Scalar ry, Scalar rz, Scalar vx, Scalar vy, Scalar vz) {
    m_coeffs[0] = rx;
    m_coeffs[1] = ry;
    m_coeffs[2] = rz;
    m_coeffs[3] = vx;
    m_coeffs[4] = vy;
    m_coeffs[5] = vz;
  }
  /** Constructs a element of se(3) from an element of so(3) \c r and R^3 \c v */
  inline LieAlgebra(const typename Base::so3Element& r, const typename Base::Vector3& v) {
    this->getR3Element() = v;
    this->getso3Element() = r;
  }

  /** \returns The stored coefficients */
  Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  const Coefficients& get() const { return m_coeffs; }

protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};

/***************************************************************************
* Definition/implementation of LieAlgebraDual<Matrix<Scalar, 6, 1> >
***************************************************************************/

/**
 * \brief Class for the se*(3) Lie algebra dual.  
 *
 * \tparam _Scalar the type of the underlying array
 *
 * This class is a specialization of LieAlgebraDual. It adds specific constructor for se*(3). 
 *
 * \sa The methods are defined in LieAlgebraBase
 */

template<typename _Scalar> class LieAlgebraDual<Matrix<_Scalar, 6, 1> > : 
  public LieAlgebraDualBase<Matrix<_Scalar, 6, 1>, LieAlgebraDual<Matrix<_Scalar, 6, 1> > > 
{
protected:
  typedef LieAlgebraDualBase<Matrix<_Scalar, 6, 1>, LieAlgebraDual<Matrix<_Scalar, 6, 1> > > Base;
public:
  // inherit MatrixBase operator
  EIGEN_DENSE_PUBLIC_INTERFACE(LieAlgebraDual)
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(LieAlgebraDual)

  typedef typename internal::traits<LieAlgebraDual<Matrix<Scalar, 6, 1> > >::Coefficients Coefficients;

  inline LieAlgebraDual() : m_coeffs() {}
  inline LieAlgebraDual(const LieAlgebraDual& g) : m_coeffs(g.get() ) {}
  inline LieAlgebraDual(const Matrix<Scalar, 6, 1>& g) : m_coeffs(g) {}
  inline LieAlgebraDual(Scalar rx, Scalar ry, Scalar rz, Scalar vx, Scalar vy, Scalar vz) {
    m_coeffs << rx, ry, rz, vx, vy, vz;
  }
  inline LieAlgebraDual(const typename Base::so3Element& r, const typename Base::Vector3& v) {
    this->getR3Element() = v;
    this->getso3Element() = r;
  }

  inline Coefficients& get() { return m_coeffs; }
  inline const Coefficients& get() const { return m_coeffs; }

protected:
  Coefficients m_coeffs;
};

#endif
