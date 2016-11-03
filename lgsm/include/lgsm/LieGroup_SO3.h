// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_LIE_GROUP_SO3_H
#define EIGEN_LGSM_LIE_GROUP_SO3_H

/***************************************************************************
* Definition/implementation of LieGroupBase<Quaternion, Derived>
***************************************************************************/

/**
 * LieGroupBase<Quaternion, Derived>
 *
 * \brief Base class for SO(3) Lie Group. 
 *
 * \tparam Derived the derived class holding the coefficients which are of type Quaternion<Scalar> or Map<Quaternion<Scalar> >
 *
 * This class actually implements methods form LieGroupBase for SO(3)
 */

template<class Derived> 
class LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived> 
{
public:
  /* List of typedef */  
  /** The type of stored coefficients */
  typedef typename internal::traits<Derived>::Scalar Scalar;
  /** The wrapped class */
  typedef Quaternion<Scalar> BaseType;
  /** The kind of stored coefficients */
  typedef typename internal::traits<Derived>::Coefficients Coefficients;
  /** The plain object returned, while using Map<LieGroup< > >*/
  typedef LieGroup<Quaternion<Scalar> > PlainObject;

  /** The type of the adjoint Matrix */
  typedef Matrix<Scalar, 3, 3> AdjointMatrix;   
  /** The type of the associated Algebra */   
  typedef LieAlgebra<Matrix<Scalar, 3, 1> > Algebra;  
  /** The type of the dual Algebra */
  typedef LieAlgebraDual<Matrix<Scalar, 3, 1> > AlgebraDual;

  /** \returns a plain object describing the inverse element*/
  EIGEN_STRONG_INLINE PlainObject inverse() const;
  /** \returns The plain object describing the identity element of the Lie group*/
  static PlainObject Identity();
  /** \returns The plain object describing the composition of two Base elements*/
  template<class OtherDerived> EIGEN_STRONG_INLINE PlainObject operator*(const LieGroupBase<BaseType, OtherDerived>& other) const;
  /** \returns the compose in place*/
  template<class OtherDerived> EIGEN_STRONG_INLINE Derived& operator*=(const LieGroupBase<BaseType, OtherDerived>& other);
  /** \returns The transformation of \c v through the rotation described by this element */
  template<class MatrixDerived> EIGEN_STRONG_INLINE Matrix<Scalar, 3, 1> operator*(const MatrixBase<MatrixDerived>& v) const {
    return this->get() * v;
  }

  /** \returns The matrix which is the adjoint representation of the element */
  inline AdjointMatrix adjoint(void) const;
  /** \returns The algebra which is the product of the adjoint representation and a element of the associated algebra */
  template<class AlgebraDerived> inline Algebra adjoint(const LieAlgebraBase<Matrix<Scalar, 3, 1>, AlgebraDerived>& ) const;           // not implemented yet
  /** \returns The algebra dual which is the product of the adjoint representation and a element of the associated algebra dual */
  template<class AlgebraDualDerived> inline AlgebraDual adjointTr(const LieAlgebraDualBase<Matrix<Scalar, 3, 1>, AlgebraDualDerived>& ) const; // not implemented yet

  /** \returns The element of the associated Algebra through the exponential map 
   *  \sa the exponential map is given by LieAlgebra::exp()
   */
  Algebra log(const Scalar precision = 1e-6) const;
  
  /** Default assignement operator */
  EIGEN_STRONG_INLINE LieGroupBase& operator=(const LieGroupBase& other);
  /** Assignement operator between derived type*/
  template<class OtherDerived> EIGEN_STRONG_INLINE Derived& operator=(const LieGroupBase<BaseType, OtherDerived>& other);
  /** Assignement operator from a Matrix*/
  template<class OtherDerived> Derived& operator=(const MatrixBase<OtherDerived>& m){ this->get() = m; return this->derived();}

  /** The read-only accessor to the derived class */
  inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
  /** The accessor to the derived class */
  inline Derived& derived() { return *static_cast<Derived*>(this); }


  /** \returns the \c x coefficient */
  inline Scalar x() const { return this->get().x(); }
  /** \returns the \c y coefficient */
  inline Scalar y() const { return this->get().y(); }
  /** \returns the \c z coefficient */
  inline Scalar z() const { return this->get().z(); }
  /** \returns the \c w coefficient */
  inline Scalar w() const { return this->get().w(); }

  /** \returns a reference to the \c x coefficient */
  inline Scalar& x() { return this->get().x(); }
  /** \returns a reference to the \c y coefficient */
  inline Scalar& y() { return this->get().y(); }
  /** \returns a reference to the \c z coefficient */
  inline Scalar& z() { return this->get().z(); }
  /** \returns a reference to the \c w coefficient */
  inline Scalar& w() { return this->get().w(); }

    /** \returns \c *this with scalar type casted to \a NewScalarType
    *
    * Note that if \a NewScalarType is equal to the current scalar type of \c *this
    * then this function smartly returns a const reference to \c *this.
    */
  template<typename NewScalarType>
  inline typename internal::cast_return_type<Derived, LieGroup<Quaternion<NewScalarType> > >::type cast() const
  {
    return typename internal::cast_return_type<Derived, LieGroup<Quaternion<NewScalarType> > >::type(
     get().template cast<NewScalarType>());
  }

  /** \returns a read-only vector expression of the imaginary part (x,y,z) */
  inline const VectorBlock<Coefficients,3> vec() const { return this->get().vec(); }

  /** \returns a vector expression of the imaginary part (x,y,z) */
  inline VectorBlock<Coefficients,3> vec() { return this->get().vec(); }

  /** \returns The stored coefficients by the derived class*/
  Coefficients& get() { return derived().get(); }
  /** \returns The read-only access to the stored coefficients by the derived class*/
  const Coefficients& get() const { return derived().get(); }
};

/*****************************************************
 * Implementation of LieGroupBase<Quaternion> methods
 *****************************************************/

// assignation
template<class Derived>
EIGEN_STRONG_INLINE LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>& 
 LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::operator=(const LieGroupBase& other) 
{
  this->get() = other.get();
  return *this;
}

//assignation
template<class Derived>
template<class OtherDerived>
EIGEN_STRONG_INLINE Derived& 
LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::operator=(const LieGroupBase<BaseType, OtherDerived>& other)
{
  this->get() = other.get();
  return derived();
}

// inverse
template<class Derived>
EIGEN_STRONG_INLINE typename LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::PlainObject
 LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::inverse() const 
{
  return PlainObject(this->get().conjugate());
}

// identity
template<class Derived>
typename LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::PlainObject 
 LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::Identity() 
{
  return PlainObject(Quaternion<Scalar>::Identity());
}

// composition
template<class Derived>
template<class OtherDerived> EIGEN_STRONG_INLINE typename LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::PlainObject 
 LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::operator*(const LieGroupBase<Quaternion<Scalar>, OtherDerived>& other) const 
{
  return PlainObject(this->get() * other.get());
}

// composition
template<class Derived>
template<class OtherDerived>
EIGEN_STRONG_INLINE Derived& 
LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::operator*=(const LieGroupBase<BaseType, OtherDerived>& other)
{
  this->get() *= other.get();
  return derived();
}

//log
template<class Derived>
typename  LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::Algebra  
 LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::log(const Scalar precision) const 
{
  const Scalar n2 = this->get().vec().squaredNorm();
  const Scalar n = std::sqrt(n2);

  if (n < precision) 
    return Algebra((2 / this->get().w()) * this->get().vec());
  else 
    return Algebra(std::atan2(2 * n * this->get().w(), this->get().w() * this->get().w() - n2) / n * this->get().vec());
}

// adjoint
template<class Derived>
typename LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::AdjointMatrix 
 LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>::adjoint() const 
{
  return this->get().toRotationMatrix();
}

template<class Derived>
inline std::ostream& operator <<(std::ostream& os, const LieGroupBase<Quaternion<typename internal::traits<Derived>::Scalar>, Derived>& g)
{
  os << g.w() << "\t" << g.x() << "\t" << g.y() << "\t" << g.z();
  return os;
}

/***************************************************************************
* Definition/implementation of LieGroup<quaternion<Scalar> >
***************************************************************************/

/**
 * Definition of LieGroup<Quaternion>
 *
 * \brief Class for SO(3) Lie Group.
 *
 * \tparam _Scalar the type of the underlying quaternion
 *
 * This class is a specialization of LieGroup. It adds specific constructor for SO(3). 
 *
 * \sa The methods are defined in LieGroupBase
 */

template<typename _Scalar> class LieGroup<Quaternion<_Scalar> > : 
  public LieGroupBase<Quaternion<_Scalar>, LieGroup<Quaternion<_Scalar> > > 
{
protected:
  /** The inherited class */
  typedef LieGroupBase<Quaternion<_Scalar>, LieGroup<Quaternion<_Scalar> > > Base;
public:
  /** The coefficients type*/
  typedef _Scalar Scalar;
  /** the stored coefficients */
  typedef typename internal::traits<LieGroup<Quaternion<Scalar> > >::Coefficients Coefficients;
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(LieGroup)

  /** Default constructor */
  inline LieGroup() : m_coeffs() {}
  /** Copy constructor **/
  template<class OtherDerived> inline LieGroup(const LieGroupBase<typename Base::BaseType, OtherDerived>& g) : m_coeffs(g.get()) {}
  /** Copy constructor */
  //inline LieGroup(const LieGroup& g) : m_coeffs(g.get() ) {}
  /** Copy constructor */
  EIGEN_STRONG_INLINE LieGroup(const Coefficients& g) : m_coeffs(g) {}
  EIGEN_STRONG_INLINE LieGroup(const AngleAxis<Scalar>& aa) : m_coeffs(aa) {}
  /** Constructs a element of SO(3) from a scalar \c w and a vector \c vec. The underlying quaternion is
   * initialized with w, vec[0], vec[1], vec[2] */
  template<class OtherDerived> inline LieGroup(Scalar w, const MatrixBase<OtherDerived>& vec) 
    : m_coeffs(w, vec.coeff(0), vec.coeff(1), vec.coeff(2))
  {
    // [XXX] check other size
  }

  /** Intialize from rotation matrix */
  template<typename Derived>
  explicit inline LieGroup(const MatrixBase<Derived> & other) { this->get() = other;}

  template<typename Derived>
  explicit inline LieGroup(const Matrix<Scalar, 4, 1> & other) { this = other;}

  /** Constructs and initializes the quaternion \f$ w+xi+yj+zk \f$ from
    * its four coefficients \a w, \a x, \a y and \a z.
    *
    * \warning Note the order of the arguments: the real \a w coefficient first,
    * while internally the coefficients are stored in the following order:
    * [\c x, \c y, \c z, \c w]
    */
  inline LieGroup(Scalar w, Scalar x, Scalar y, Scalar z) : m_coeffs(w, x, y, z) {}
  inline LieGroup(Scalar w, const Matrix<Scalar, 3, 1> & v) : m_coeffs(w, v.x(), v.y(), v.z()) {}

  /** \returns The stored coefficients */
  Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  const Coefficients& get() const { return m_coeffs; }

protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};

/***************************************************************************
* Definition/implementation of Map<LieGroup<Quaternion<Scalar> > >
***************************************************************************/

// we don't need anything new

#endif
