// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_DISPLACEMENT_H
#define EIGEN_LGSM_DISPLACEMENT_H

/*******************************************************************************
* Definition/implementation of DisplacementBase
********************************************************************************/

/**
 * \class DisplacementBase
 *
 * \brief Base class describing a rigid Displacement or a 3D Frame position. 
 *
 * \tparam Derived the derived class holding the coefficients which are of type Array<Scalar, 7, 1> or Map<Array<Scalar, 7, 1> >
 *
 * This class abstracts the underlying mathematical definition and add some accessors. It uses an Array internally to store its coefficients.
 */

template<class Derived> 
class DisplacementBase : public LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1>, Derived>{
public:
  /** The coefficients type*/
  typedef typename internal::traits<Derived>::Scalar Scalar;
protected:
  /** The inherited class */
  typedef LieGroupBase<Array<Scalar, 7, 1>, Derived> Base;
public:
  // inherit the operator=
  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(DisplacementBase<Derived>)

  /** The wrapped class (Array<Scalar, 7, 1> */
  typedef typename Base::BaseType BaseType;
  /** The plain object returned, while using Map<Displacement >*/
  typedef typename Base::PlainObject PlainObject;

  /** The type of the translation part*/ 
  typedef Matrix<Scalar, 3, 1> Vector3;
  /** The type of the rotation part*/ 
  typedef LieGroup<Quaternion<Scalar> > Rotation3D;

  /** The accessor to the rotation part */
  inline Map<Rotation3D> getRotation(){ return Map<Rotation3D>(this->derived().get().template head<4>().data()); }
  /** The read-only accessor to the rotation part */
  inline const Map<const Rotation3D> getRotation() const{ return Map<const Rotation3D>(this->derived().get().template head<4>().data()); }
  /** The accessor to the translation part */
  inline Map<Vector3> getTranslation(){ return Map<Vector3>(this->derived().get().template tail<3>().data()); }
  /** The read-only accessor to the translation part */
  inline const Map<const Vector3> getTranslation() const { return Map<const Vector3>(this->get().template tail<3>().data()); }


  /** \returns the \c x coefficient */
  inline Scalar x() const { return this->getTranslation().x(); }
  /** \returns the \c y coefficient */
  inline Scalar y() const { return this->getTranslation().y(); }
  /** \returns the \c z coefficient */
  inline Scalar z() const { return this->getTranslation().z(); }
  /** \returns the \c qx coefficient */
  inline Scalar qx() const { return this->getRotation().x(); }
  /** \returns the \c qy coefficient */
  inline Scalar qy() const { return this->getRotation().y(); }
  /** \returns the \c qz coefficient */
  inline Scalar qz() const { return this->getRotation().z(); }
  /** \returns the \c qw coefficient */
  inline Scalar qw() const { return this->getRotation().w(); }

  /** \returns a reference to the \c x coefficient */
  inline Scalar& x() { return this->getTranslation().x(); }
  /** \returns a reference to the \c y coefficient */
  inline Scalar& y() { return this->getTranslation().y(); }
  /** \returns a reference to the \c z coefficient */
  inline Scalar& z() { return this->getTranslation().z(); }
  /** \returns a reference to the \c qx coefficient */
  inline Scalar& qx() { return this->getRotation().x(); }
  /** \returns a reference to the \c qy coefficient */
  inline Scalar& qy() { return this->getRotation().y(); }
  /** \returns a reference to the \c qz coefficient */
  inline Scalar& qz() { return this->getRotation().z(); }
  /** \returns a reference to the \c qw coefficient */
  inline Scalar& qw() { return this->getRotation().w(); }

  inline void setRandom() {
    this->get().setRandom();
    this->getRotation().get().normalize();
  }
  
  /** \returns \c *this with scalar type casted to \a NewScalarType
   *
   * Note that if \a NewScalarType is equal to the current scalar type of \c *this
   * then this function smartly returns a const reference to \c *this.
   */
  template<typename NewScalarType>
  inline typename internal::cast_return_type<Derived, Displacement<NewScalarType> >::type cast() const
  {
    return typename internal::cast_return_type<Derived, Displacement<NewScalarType> >::type(
     this->get().template cast<NewScalarType>());
  }

  EIGEN_STRONG_INLINE DisplacementBase& operator=(const typename Base::PlainObject& g) //[XXX] Plain object must be a Displacement
  {
    this->derived().get() = g.get();
    return *this;
  }
  
  Matrix<Scalar, 4, 4> toHomogeneousMatrix() const;

  /** Outputs to the given stream : order : w x y z (it's different from the storage order) */
  template<class OtherDerived>
  friend std::ostream& operator <<(std::ostream& os, const DisplacementBase<OtherDerived>& d);
};

template<class Derived> Matrix<typename internal::traits<Derived>::Scalar, 4, 4> DisplacementBase<Derived>::toHomogeneousMatrix() const
{
	Matrix<Scalar, 4, 4> m;
	//m << this->getRotation().get().toRotationMatrix(), Eigen::Matrix<Scalar, 3, 1>::Zero(), this->getTranslation().transpose(), 1.;
	m << this->getRotation().get().toRotationMatrix(), this->getTranslation(), Eigen::Matrix<Scalar, 1, 3>::Zero(), 1.;

	return m;
}

template<class Derived>
inline std::ostream& operator <<(std::ostream& os, const DisplacementBase<Derived>& d)
{
  os << d.x() << "\t" << d.y() << "\t" << d.z() << "\t" << d.qw() << "\t" << d.qx() << "\t" << d.qy() << "\t" << d.qz();
  return os;
}

/*********************************
 * Implementation of Displacement
 *********************************/

namespace internal {
  template<typename _Scalar>
    struct traits<Displacement<_Scalar> > : traits<LieGroup<Array<_Scalar, 7, 1> > >
    {
      typedef Displacement<_Scalar> PlainObject;
      typedef _Scalar Scalar;
    };
}

/**
 * \brief Class describing a rigid Displacement or a 3D Frame position. 
 *
 * \tparam _Scalar the type of the underlying array
 *
 * This class add some specific constructors
 *
 * \sa The methods are defined in LieGroupBase and DisplacementBase
 */

template<typename _Scalar>
class Displacement : public DisplacementBase<Displacement<_Scalar> >{
public:
  /** The coefficients type*/
  typedef _Scalar Scalar;
protected:
  /** The inherited class */
  typedef DisplacementBase<Displacement<Scalar> > Base;
public:
  /** the stored coefficients */
  typedef typename internal::traits<Displacement>::Coefficients Coefficients;
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Displacement<Scalar>)

  /** Default constructor */
  inline Displacement() : m_coeffs() {}
  /** Copy constructor */
  inline Displacement(const Displacement& other) : m_coeffs(other.get() ) {}
  //EIGEN_STRONG_INLINE Displacement(const typename Base::PlainObject& g) : m_coeffs(g.get() ) {}  // [XXX] Plain object must be a Displacement
  /** Pseudo-copy constructor */
  template<typename Derived>
  inline Displacement(const DisplacementBase<Derived>& other) : m_coeffs(other.get() ) {}
  /** Copy constructor using the wrapped class*/
  inline Displacement(const Array<Scalar, 7, 1>& g) : m_coeffs(g) {}
  template<typename Derived>
  explicit inline Displacement(const MatrixBase<Derived>& other) { *this = other; }
  /** Constructs and initializes the displacement with \f$R^3\f$ first then \f$SO(3)\f$
  *
  * \warning Note the order of the arguments: R^3 first then \c qw (scalar part)
  * while internally the coefficients are stored in the following order:
  * [\c qx, \c qy, \c qz, \c qw \c x \c y \c z]
  */
  EIGEN_STRONG_INLINE Displacement(Scalar x, Scalar y, Scalar z, Scalar qw = (Scalar)1.0, Scalar qx = (Scalar)0.0, Scalar qy = (Scalar)0.0, Scalar qz = (Scalar)0.0) { 
    // m_coeffs << qx, qy, qz, qw, x, y, z // operator<< is not inlined
    m_coeffs[0] = qx;
    m_coeffs[1] = qy;
    m_coeffs[2] = qz;
    m_coeffs[3] = qw;
    m_coeffs[4] = x;
    m_coeffs[5] = y;
    m_coeffs[6] = z;
  }


  /** Constructs a element of SE(3) from an element of SO(3) \c r and R^3 \c v */
  EIGEN_STRONG_INLINE Displacement(const typename Base::Vector3& v, const typename Base::Rotation3D& r/* = Base::Rotation3D::Identity()*/) {
    this->getTranslation() = v;
    this->getRotation() = r;
  }

  /** \returns The stored coefficients */
  inline Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  inline const Coefficients& get() const { return m_coeffs; }

protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};

/** single precision displacement type */
typedef Displacement<double> Displacementd;
/** double precision displacement type */
typedef Displacement<float> Displacementf;

/**************************************
 * Implementation of Map<Displacement>
 **************************************/

namespace internal {
  template<typename _Scalar, int MapOptions, typename StrideType>
    struct traits<Map<Displacement<_Scalar>, MapOptions, StrideType> > : Map<LieGroup<Array<_Scalar, 7, 1> >, MapOptions, StrideType>
    {
      typedef Displacement<_Scalar> PlainObject;
      typedef _Scalar Scalar;
    };
}
/**
 * \brief Class map an array to a rigid Displacement or a 3D Frame position. 
 *
 * \tparam _Scalar the type of the underlying array
 * \tparam MapOptions \see Map<Matrix>
 * \tparam StrideType \see Map<Matrix>
 *
 * \sa The methods are defined in LieGroupBase and DisplacementBase
 */

template<typename _Scalar, int MapOptions, typename StrideType>
class Map<Displacement<_Scalar>, MapOptions, StrideType> : public DisplacementBase<Map<Displacement<_Scalar>, MapOptions, StrideType> >{
  protected:
    typedef _Scalar Scalar;
    typedef DisplacementBase<Map<Displacement<Scalar> > > Base;
  public:

    EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(Map)
      typedef typename internal::traits<Map>::Coefficients Coefficients;

    inline Map(const Displacement<_Scalar>& d) : m_coeffs(d.get()) {};
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

