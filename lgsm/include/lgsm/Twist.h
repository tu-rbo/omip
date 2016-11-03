// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_LIE_TWIST_H
#define EIGEN_LGSM_LIE_TWIST_H

/*******************************************************************************
* Definition/implementation of TwistBase
********************************************************************************/

/**
 * \class TwistBase
 *
 * \brief Base class describing a Twist. 
 *
 * \tparam Derived the derived class holding the coefficients which are of type Matrix<Scalar, 6, 1> or Map<Matrix<Scalar, 6, 1> >
 *
 * This class abstracts the underlying mathematical definition and add some accessors. A twist has two part, an angular velocity and a linear velocity.
 * It uses an Matrix internally to store its coefficients.
 */

namespace internal {
  template<class Derived>
    struct traits<TwistBase<Derived> > : traits<LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived> > {
    };
}

template<class Derived> 
class TwistBase : public LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived>{
protected:
  /** The inherited class */
  typedef LieAlgebraBase<Matrix<typename internal::traits<Derived>::Scalar, 6, 1>, Derived> Base;
public:
  // inherit MatrixBase interface
  EIGEN_DENSE_PUBLIC_INTERFACE(TwistBase)
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(TwistBase)

  /** The wrapped class */
  typedef typename Base::BaseType BaseType;
  /** The kind of stored coefficients */
  typedef typename internal::traits<Derived>::Coefficients Coefficients;

  /** \returns the \c rx coefficient */
  inline Scalar rx() const { return this->getAngularVelocity().x(); }
  /** \returns the \c ry coefficient */
  inline Scalar ry() const { return this->getAngularVelocity().y(); }
  /** \returns the \c rz coefficient */
  inline Scalar rz() const { return this->getAngularVelocity().z(); }
  /** \returns the \c vx coefficient */
  inline Scalar vx() const { return this->getLinearVelocity().x(); }
  /** \returns the \c vy coefficient */
  inline Scalar vy() const { return this->getLinearVelocity().y(); }
  /** \returns the \c vz coefficient */
  inline Scalar vz() const { return this->getLinearVelocity().z(); }

  /** \returns a reference to the \c rx coefficient */
  inline Scalar& rx() { return this->getAngularVelocity().x(); }
  /** \returns a reference to the \c ry coefficient */
  inline Scalar& ry() { return this->getAngularVelocity().y(); }
  /** \returns a reference to the \c rz coefficient */
  inline Scalar& rz() { return this->getAngularVelocity().z(); }
  /** \returns a reference to the \c vx coefficient */
  inline Scalar& vx() { return this->getLinearVelocity().x(); }
  /** \returns a reference to the \c vy coefficient */
  inline Scalar& vy() { return this->getLinearVelocity().y(); }
  /** \returns a reference to the \c vz coefficient */
  inline Scalar& vz() { return this->getLinearVelocity().z(); }

  /** The type of the linear part */ 
  typedef Matrix<Scalar, 3, 1> LinearVelocity;
  /** The type of the angular part */ 
  typedef LieAlgebra<Matrix<Scalar, 3, 1> > AngularVelocity;
  
  /** The accessor to the angular velocity */
  inline Map<AngularVelocity> getAngularVelocity(){ return Map<AngularVelocity>(this->derived().get().template head<3>().data()); }
  /** The read-only accessor to the angular velocity */
  inline Map<const AngularVelocity> getAngularVelocity() const {return  Map<const AngularVelocity>(this->derived().get().template head<3>().data()); }
  /** The accessor to the linear velocity */
  inline Map<LinearVelocity> getLinearVelocity() { return  Map<LinearVelocity>(this->derived().get().template tail<3>().data()); }
  /** The read-only accessor to the angular velocity */
  inline Map<const LinearVelocity> getLinearVelocity() const { return  Map<const LinearVelocity>(this->derived().get().template tail<3>().data()); }

  template<class RotationDerived> inline Twist<Scalar> changeFrame(const LieGroupBase<Quaternion<Scalar>, RotationDerived>&) const;

  template<class OtherDerived> inline Twist<Scalar> changePoint(const MatrixBase<OtherDerived>& point) const;
};


template<class Derived>
template<class RotationDerived> 
inline Twist<typename internal::traits<TwistBase<Derived> >::Scalar> TwistBase<Derived>::changeFrame(const LieGroupBase<Quaternion<Scalar>, RotationDerived>& rot) const {
  return Twist<Scalar>(rot*this->getAngularVelocity(),
                       rot*this->getLinearVelocity());
}

template<class Derived>
template<class OtherDerived> 
inline Twist<typename internal::traits<TwistBase<Derived> >::Scalar> TwistBase<Derived>::changePoint(const MatrixBase<OtherDerived>& point) const {
  return Twist<Scalar>(this->getAngularVelocity(),
                       this->getLinearVelocity() + this->getAngularVelocity().cross(point) );
}

/**************************
 * Implementation of Twist
 **************************/

namespace internal {  
  template<typename Scalar>
    struct traits<Twist<Scalar> > 
    : public traits<LieAlgebra<Matrix<Scalar, 6, 1> > > 
    {
      typedef Displacement<Scalar> Group;
    };

  template<typename Scalar, int Options>
    struct traits<Map<Twist<Scalar>, Options> > 
    : public traits<Map<LieAlgebra<Matrix<Scalar, 6, 1> >, Options > > 
    {
      typedef Displacement<Scalar> Group;
    };
}

/**
 * \brief Class describing a Twist. 
 *
 * \tparam _Scalar the type of the underlying array
 *
 * This class add some specific constructors
 *
 * \sa The methods are defined in LieAlgebraBase and TwistBase
 */

template<typename _Scalar>
class Twist : public TwistBase<Twist<_Scalar> >{
protected:
  typedef TwistBase<Twist<_Scalar> > Base;
public:
  // inherit MatrixBase interface
  EIGEN_DENSE_PUBLIC_INTERFACE(Twist)
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_OPERATORS(Twist)

  /** The inherited class */
  typedef typename Base::BaseType BaseType;
  /** the stored coefficients */
  typedef typename internal::traits<Twist>::Coefficients Coefficients;


  /** Default constructor */
  inline Twist() : m_coeffs() {}
  /** Copy constructor */
  inline Twist(const Twist& g) : m_coeffs(g.get() ) {}
  /** Copy constructor */
  inline Twist(const BaseType& g) : m_coeffs(g) {}
  /** Constructs an element of se(3) from 6 scalar */
  inline Twist(Scalar rx, Scalar ry, Scalar rz, Scalar vx, Scalar vy, Scalar vz) {
    m_coeffs[0] = rx;
    m_coeffs[1] = ry;
    m_coeffs[2] = rz;
    m_coeffs[3] = vx;
    m_coeffs[4] = vy;
    m_coeffs[5] = vz;
  }
  /** Copy constructor : need to build a twist from a CWiseBinaryOp or CWiseUnaryOp*/
  template<typename OtherDerived>
  EIGEN_STRONG_INLINE Twist(const MatrixBase<OtherDerived>& other)
    : m_coeffs(other)
  {
  }
  /** Assignement operator : need to copy a CWiseBinaryOp or CWiseUnaryOp to a Twist*/
  //template<typename OtherDerived>
  //EIGEN_STRONG_INLINE Twist& operator=(const MatrixBase<OtherDerived>& other) 
  //{
  //  this->m_coeffs = other;
  //  return *this;
  //}
  /** Constructs Twist from an angular velocity \c r and a linear velocity \c v */
  template<class LinearVelocityDerived, class AngularVelocityDerived>
  inline Twist(const LieAlgebraBase<Matrix<Scalar, 3, 1>, AngularVelocityDerived>& r, const MatrixBase<LinearVelocityDerived>& v) {
    this->getLinearVelocity() = v;
    this->getAngularVelocity() = r;
  }
  /** Constructs Twist from an angular velocity \c r and a linear velocity \c v */
  inline Twist(const typename Base::AngularVelocity& r, const typename Base::LinearVelocity& v) {
    this->getLinearVelocity() = v;
    this->getAngularVelocity() = r;
  }
  /** \returns The stored coefficients */
  inline Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const { return m_coeffs; }

protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};

/** single precision twist type */
typedef Twist<double> Twistd;
/** double precision twist type */
typedef Twist<float> Twistf;

/**************************************
 * Implementation of Map<Twist>
 **************************************/


/**
 * \brief Class map an array to twist. 
 *
 * \tparam _Scalar the type of the underlying array
 * \tparam MapOptions \see Map<Matrix>
 * \tparam StrideType \see Map<Matrix>
 *
 * \sa The methods are defined in LieAlgebraBase and TwistBase
 */


template<typename _Scalar, int MapOptions, typename StrideType>
class Map<Twist<_Scalar>, MapOptions, StrideType> : public TwistBase<Map<Twist<_Scalar>, MapOptions, StrideType> >{
  protected:
    typedef TwistBase<Map<Twist<_Scalar> > > Base;
  public:
    EIGEN_DENSE_PUBLIC_INTERFACE(Map)
      EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)
      typedef typename internal::traits<Map>::Coefficients Coefficients;

    inline Map(const Twist<Scalar>& d) : m_coeffs(d.get()) {};
    template<int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols> 
      inline Map(const Array<Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& g) : m_coeffs(g.data()) {};

    inline Map(Scalar* data) : m_coeffs(data) {};
    inline Map(const Map& m) : m_coeffs(m.get()) {};

    inline Coefficients& get() { return m_coeffs; }
    inline const Coefficients& get() const { return m_coeffs; }

  protected:
    Coefficients m_coeffs;
};

#endif

