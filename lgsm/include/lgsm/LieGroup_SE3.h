// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_LIE_SE3_H
#define EIGEN_LGSM_LIE_SE3_H

namespace internal {
template<typename Other,
         int OtherRows=Other::RowsAtCompileTime,
         int OtherCols=Other::ColsAtCompileTime>
struct liegroup_SE3_base_assign_impl;
}

/*******************************************************************************
* Definition/implementation of LieGroupBase<Array<Scalar, 7, 1, Derived>
********************************************************************************/

/**
 * LieGroupBase<Array<Scalar, 7, 1>, Derived>
 *
 * \brief Base class for SE(3) Lie Group. 
 *
 * \tparam Derived the derived class holding the coefficients which are of type Array<Scalar, 7, 1> or Map<Array<Scalar, 7, 1> >
 *
 * This class actually implements the methods defined in LieGroupBase<G> and add specific accessors. Since Se(3) is the semi direct product of R^3 
 * and SO(3) many operations ar eperformed using directly the part form R^3 or SO(3)
 */


template<class Derived> 
class LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1>, Derived> 
{
public:
  /* List of typedef */  
  /** The type of stored coefficients */
  typedef typename internal::traits<Derived>::Scalar Scalar;
  /** The wrapped class */
  typedef Array<Scalar, 7, 1> BaseType;
  /** The kind of stored coefficients */
  typedef typename internal::traits<Derived>::Coefficients Coefficients;
  /** The plain object returned, while using Map<LieGroup< > >*/
  typedef typename internal::traits<Derived>::PlainObject PlainObject;

  /** The type of the adjoint Matrix */
  typedef Matrix<Scalar, 6, 6> AdjointMatrix;  
  /** The type of the associated Algebra */     
  typedef LieAlgebra<Matrix<Scalar, 6, 1> > Algebra;   
  /** The type of the dual Algebra */ 
  typedef LieAlgebraDual<Matrix<Scalar, 6, 1> > AlgebraDual;

  /** The type of an element of R^3*/ 
  typedef Matrix<Scalar, 3, 1> Vector3;
  /** The type of an element of SO(3) */ 
  typedef LieGroup<Quaternion<Scalar> > SO3Element;  // an element of SO(3)

  /** \returns a plain object describing the inverse element*/
  EIGEN_STRONG_INLINE PlainObject inverse() const;
  /** \returns The plain object describing the identity element of the Lie group*/
  static PlainObject Identity();
  /** \returns The plain object describing the composition of two Base elements*/
  template<class OtherDerived> inline PlainObject operator*(const LieGroupBase<BaseType, OtherDerived>& other) const;
  /** \returns the compose in place*/
  template<class OtherDerived> EIGEN_STRONG_INLINE Derived& operator*=(const LieGroupBase<BaseType, OtherDerived>& other);
  /** \returns The transformation of \c v through the rigid displacement described by this element */
  template<class OtherDerived> inline Vector3 operator*(const MatrixBase<OtherDerived>& d) const;

  /** \returns The matrix which is the adjoint representation of the element */
  AdjointMatrix adjoint(void) const;
  /** \returns The algebra which is the product of the adjoint representation and a element of the associated algebra */
  template<class AlgebraDerived> inline Algebra adjoint(const LieAlgebraBase<Matrix<Scalar, 6, 1>, AlgebraDerived>& ) const;
  /** \returns The algebr dual which is the product of the adjoint representation and a element of the associated algebra dual */
  template<class AlgebraDualDerived> inline AlgebraDual adjointTr(const LieAlgebraDualBase<Matrix<Scalar, 6, 1>, AlgebraDualDerived>& ) const;
  
  /** \returns The element of the associated Algebra through the exponential map 
   *  \sa the exponential map is given by LieAlgebra::exp()
   */
  inline Algebra log(const Scalar precision = 1e-6) const;
  
  /** Default assignement operator */
  EIGEN_STRONG_INLINE LieGroupBase& operator=(const LieGroupBase& other);
  /** Assignement operator between derived type*/
  template<class OtherDerived> EIGEN_STRONG_INLINE Derived& operator=(const LieGroupBase<BaseType, OtherDerived>& other);
  /** Assignement operator from a Matrix*/
  template<class OtherDerived> Derived& operator=(const MatrixBase<OtherDerived>& m);

  /** The read-only accessor to the derived class */
  inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
  /** The accessor to the derived class */
  inline Derived& derived() { return *static_cast<Derived*>(this); }

  /** The accessor to the SO(3) element */
  inline Map<SO3Element> getSO3Element(){ return Map<SO3Element>(this->derived().get().template head<4>().data()); }
  /** The read-only accessor to the SO(3) element */
  inline Map<const SO3Element> getSO3Element() const{ return Map<const SO3Element>(this->derived().get().template head<4>().data()); }
  /** The accessor to the R^3 element */
  // a map is prefered to a VectorBlock, because the underlying data is an array and not a matrix
  inline Map<Vector3> getR3Element(){ return Map<Vector3>(this->get().template tail<3>().data()); }
  /** The read-only accessor to the R^3 element */
  inline Map<const Vector3> getR3Element() const { return Map<const Vector3>(this->get().template tail<3>().data()); }

  /** \returns The stored coefficients by the derived class*/
  Coefficients& get() { return derived().get(); }
  /** \returns The read-only access to the stored coefficients by the derived class*/
  const Coefficients& get() const { return derived().get(); }  
  
  /** Outputs to the given stream : order : x y z qw qx qy qz (it's different from the storage order) */
  template<class OtherDerived>
  friend std::ostream& operator <<(std::ostream& os, const LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1>, Derived>& g);
};

/***************************************************************
 * Implementation of LieGroupBase<Array<Scalar, 7, 1> > methods
 ***************************************************************/

// assignation
template<class Derived>
inline LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>& 
  LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::operator=(const LieGroupBase& other) 
{
  this->get() = other.get();
  return *this;
}

template<class Derived>
template<class OtherDerived>
inline Derived& 
  LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::operator=(const LieGroupBase<BaseType, OtherDerived>& other) 
{
  this->get() = other.get();
  return derived();
}

template<class Derived>
template<class MatrixDerived>
inline Derived& 
  LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::operator=(const MatrixBase<MatrixDerived>& xpr)
  {
    EIGEN_STATIC_ASSERT((internal::is_same<typename Derived::Scalar, typename MatrixDerived::Scalar>::value),
   YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
  
   internal::liegroup_SE3_base_assign_impl<MatrixDerived>::run(*this, xpr.derived());

   return derived();
  }
// inverse
template<class Derived>
typename LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::PlainObject
  LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::inverse() const 
{
     return PlainObject(-(this->getSO3Element().inverse().operator*(this->getR3Element()))  // -rot^-1*trans
                        , this->getSO3Element().inverse());                                 // rot^-1
}

// identity
template<class Derived>
typename LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::PlainObject 
  LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::Identity() 
{
  return PlainObject(Vector3(0,0,0), SO3Element::Identity());
}

// composition
//
// [ R1   p1 ] * [ R2 p2 ] = [ (R1*p2 + p1) (R1*R2) ]  
//
template<class Derived>
template<class OtherDerived> EIGEN_STRONG_INLINE 
typename LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::PlainObject 
  LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::operator*(const LieGroupBase<Array<Scalar, 7, 1 >, OtherDerived>& other) const 
{
  return PlainObject(this->getSO3Element() * other.getR3Element() + this->getR3Element()  // rot1 * trans2 + trans1
                     , this->getSO3Element() * other.getSO3Element());                         // rot1 * rot2
}


template<class Derived>
template<class OtherDerived> EIGEN_STRONG_INLINE 
 Derived& 
LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::operator*=(const LieGroupBase<BaseType, OtherDerived>& other) {

  this->getR3Element() = this->getSO3Element() * other.getR3Element() + this->getR3Element();
  this->getSO3Element() *= other.getSO3Element();

  return derived();
}

// composition
// [ R1   p1 ] * [ p2 ] = [ R1*p2 + p1 ]  
//
template<class Derived>
template<class OtherDerived> EIGEN_STRONG_INLINE 
typename LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::Vector3 
 LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::operator*(const MatrixBase<OtherDerived>& vec) const 
{
  return Vector3(this->getSO3Element() * vec + this->getR3Element());
}

// log
template<class Derived>
typename LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::Algebra 
  LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::log(const Scalar precision) const 
{
  typename SO3Element::Algebra ang = this->getSO3Element().log(precision);

  const Scalar n2 = ang.squaredNorm();  // ||wtidle||^2

  if(n2 < precision)
    return Algebra(ang, this->getR3Element());
  else{

    const Scalar n = std::sqrt(n2);

#ifndef USE_RLAB_LOG_FUNCTION   //Set in Lgsm file
    const Scalar sn = std::sin(n);

    Scalar val = (Scalar(2.0) * sn - n * (Scalar(1.0) + std::cos(n))) / (Scalar(2.0) *n2 * sn);

    Vector3 lin = -0.5*ang.cross(this->getR3Element());
    lin += (Scalar(1.0) - val * n2 ) * this->getR3Element();
    lin += val * ang.dot(this->getR3Element()) * ang;

    return Algebra(ang, lin);

#else
    // The previous code is unstable when the norm of the rotation gets close to pi
    // because we divide by sn=sin(n) that is close to zero to obtain val
    // We replace that code with code from RLab -> they work with the norm/2, sin(norm/2) and cos(norm/2)/sin(norm/2)
    // and given that norm is between pi and -pi,  sin(norm/2) is not going to be close to zero
    // (see HTransform.cpp, Vector3D.cpp and Rotation.cpp from RLab)
    const Scalar n_div2 = n/2.0;
    const Scalar s = std::sin(n_div2) / n_div2;
    const Scalar c = std::cos(n_div2);
    const Scalar gamma = c / s;

    Eigen::Matrix<double, 3, 3> dexpinv = Eigen::Matrix<double, 3, 3>::Identity();

    double v1 = ang[0];
    double v2 = ang[1];
    double v3 = ang[2];

    Eigen::Matrix<double, 3, 3> w_ceil = Eigen::Matrix<double, 3, 3>::Zero();
    w_ceil(0, 1) = -v3;
    w_ceil(0, 2) = v2;
    w_ceil(1, 0) = v3;
    w_ceil(1, 2) = -v1;
    w_ceil(2, 0) = -v2;
    w_ceil(2, 1) = v1;

    Eigen::Matrix<double, 3, 3> w_ceil_sqr = Eigen::Matrix<double, 3, 3>::Zero();
    double v1sqr = v1 * v1;
    double v2sqr = v2 * v2;
    double v3sqr = v3 * v3;
    double v12 = v1 * v2;
    double v13 = v1 * v3;
    double v23 = v2 * v3;

    w_ceil_sqr(0, 0) = -v2sqr - v3sqr;
    w_ceil_sqr(0, 1) = v12;
    w_ceil_sqr(0, 2) = v13;
    w_ceil_sqr(1, 0) = v12;
    w_ceil_sqr(1, 1) = -v1sqr - v3sqr;
    w_ceil_sqr(1, 2) = v23;
    w_ceil_sqr(2, 0) = v13;
    w_ceil_sqr(2, 1) = v23;
    w_ceil_sqr(2, 2) = -v1sqr - v2sqr;

    dexpinv += -0.5 * w_ceil + (1 - gamma) / n2 * w_ceil_sqr;

    Vector3 lin = dexpinv*this->getR3Element();

    return Algebra(ang, lin);
#endif
  }
}


//RLab code
//dMatrix dexpinv;
//dExpInv_SO3(xi.w, dexpinv);
//xi.v = dexpinv*r;

//void dExpInv_SO3(const Vector3D& w, dMatrix& dexpinv)
//{
//    double wnorm = w.Norm();

//    dexpinv.resize(3, 3);
//    dexpinv.identity();

//    if (wnorm > RMATH_ZERO)
//    {
//        double half_wnorm = wnorm / 2;
//        double s = sin(half_wnorm) / half_wnorm;
//        double c = cos(half_wnorm);

//        double gamma = c / s;
//        double wnorm_sqr = wnorm * wnorm;

//        //dMatrix temp1 = -0.5*w.Ceil(); temp1.print("-1/2*w.Ceil()");
//        //dMatrix temp2 = (1 - gamma)/wnorm_sqr*w.CeilSqr(); temp2.print("(1 - gamma)/wnorm_sqr*w.CeilSqr()");

//        dexpinv += -0.5 * w.Ceil() + (1 - gamma) / wnorm_sqr * w.CeilSqr();
//    }
//}

// adjoint
//
// Ad_H =  [ R          0 ]
//         [ ptilde*R   R ]
//
template<class Derived>
typename LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::AdjointMatrix 
  LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::adjoint() const 
{
  AdjointMatrix res;

  res.template block<3,3>(0,0) = this->getSO3Element().adjoint();
  res.template block<3,3>(3,3) = res.template block<3,3>(0,0);


  res.template block<3,3>(3,0) = (Matrix<Scalar,3,3>() <<
                                  0, -this->getR3Element()[2], this->getR3Element()[1], // 0 -z  y
                                  this->getR3Element()[2], 0, -this->getR3Element()[0], // z  0 -x 
                                  -this->getR3Element()[1], this->getR3Element()[0], 0)	//-y  x  0
                         .finished()*res.template block<3,3>(0,0);

  (res.template block<3,3>(0,3)).setZero();

  return res;
}

//
// Ad_H * T =  [ R          0 ] * [ w ] = [       R*w   ] 
//             [ ptilde*R   R ]   [ v ]   [ (p^R*w)+R*v ]
//
template<class Derived>
template<class AlgebraDerived>
inline typename LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::Algebra 
  LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::adjoint(const LieAlgebraBase<Matrix<Scalar, 6, 1>, AlgebraDerived>& a) const 
{
  Vector3 Rw = this->getSO3Element() * a.getso3Element();                       // R*w

  typename SO3Element::Algebra ang(Rw);
  Vector3 lin = this->getR3Element().cross(Rw) + this->getSO3Element() * a.getR3Element(); // (p^R*w)+R*v

  return Algebra(ang, lin);
}

//
// (Ad_H)' * W =  [ R'   -R'*ptilde ] * [ t ] = [  R'*(f^p+t) ] 
//                [ 0           R'  ]   [ f ]   [      R'*f   ]
//
template<class Derived>
template<class AlgebraDualDerived> 
inline typename LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::AlgebraDual 
  LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1 >, Derived>::adjointTr(const LieAlgebraDualBase<Matrix<Scalar, 6, 1>, AlgebraDualDerived>& ca) const 
{
  return AlgebraDual(this->getSO3Element().inverse() * (ca.getR3Element().cross(this->getR3Element() ) + ca.getso3Element()),  // R'*(f^p+t)
                   this->getSO3Element().inverse() * ca.getR3Element());                                                     // R'*f
}

/***************************************************************************
* Definition/implementation of LieGroup<Array<Scalar, 7, 1> >
***************************************************************************/

/**
 * Definition of LieGroup<Array<Scalar, 7, 1>>
 *
 * \brief Class for SE(3) Lie Group. 
 *
 * \tparam _Scalar the type of the underlying array
 *
 * This class is a specialization of LieGroup. It adds specific constructor for SE(3). 
 *
 * \sa The methods are defined in LieGroupBase
 */

template<typename _Scalar> class LieGroup<Array<_Scalar, 7, 1> > : 
  public LieGroupBase<Array<_Scalar, 7, 1>, LieGroup<Array<_Scalar, 7, 1> > > 
{
protected:
  /** The inherited class */
  typedef LieGroupBase<Array<_Scalar, 7, 1>, LieGroup<Array<_Scalar, 7, 1> > > Base;
public:
  /** The coefficients type*/
  typedef _Scalar Scalar;
  /** the stored coefficients */
  typedef typename internal::traits<LieGroup<Array<_Scalar, 7, 1> > >::Coefficients Coefficients;
  // inherit assignement operator
  EIGEN_INHERIT_ASSIGNMENT_EQUAL_OPERATOR(LieGroup)

  /** Default constructor */
  inline LieGroup() : m_coeffs() {}
  /** Copy constructor */
  inline LieGroup(const LieGroup& g) : m_coeffs(g.get() ) {}
  /** Copy constructor */
  inline LieGroup(const Array<Scalar, 7, 1>& g) : m_coeffs(g) {}

  /** Constructs and initializes the displacement with \f$R^3\f$ first then \f$SO(3)\f$
  *
  * \warning Note the order of the arguments: R^3 first then \c qw (scalar part)
  * while internally the coefficients are stored in the following order:
  * [\c qx, \c qy, \c qz, \c qw \c x \c y \c z]
  */
  inline LieGroup(Scalar x, Scalar y, Scalar z, Scalar qw, Scalar qx, Scalar qy, Scalar qz) {
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
  EIGEN_STRONG_INLINE LieGroup(const typename Base::Vector3& v, const typename Base::SO3Element& r) {
    this->getR3Element() = v;
    this->getSO3Element() = r;
  }
  template<typename Derived>
  explicit inline LieGroup(const MatrixBase<Derived>& other) { this = other;}

  /** \returns The stored coefficients */
  Coefficients& get() { return m_coeffs; }
  /** \returns The read-only access to the stored coefficients */
  const Coefficients& get() const { return m_coeffs; }

protected:
  /** The wrapped coefficients */
  Coefficients m_coeffs;
};

// set from a rotation matrix
namespace internal {
  template<typename Other>
  struct liegroup_SE3_base_assign_impl<Other,4,4>
  {
    // Other and LieGroup Scalar must have the same type (it's already checked in the calling function)
    typedef typename Other::Scalar Scalar;
    template<class Derived> inline static void run(LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1>, Derived>& q, const Other& m)
    {   
      q.getSO3Element().get() =  m.template topLeftCorner<3,3>();
      q.getR3Element() = m.template topRightCorner<3,1>();
    }
  };

  // set from a vector of coefficients assumed to be a quaternion
  template<typename Other>
  struct liegroup_SE3_base_assign_impl<Other,3,1>
  {
    // Other and LieGroup Scalar must have the same type (it's already checked in the calling function)
    typedef typename Other::Scalar Scalar;
    template<class Derived> inline static void run(LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1>, Derived>& q, const Other& vec)
    {
      q.getR3Element() = vec;
      q.getSO3Element() = LieGroupBase<Array<typename internal::traits<Derived>::Scalar, 7, 1>, Derived>::SO3Element::Identity();
    }
  };
} // namespace

/***************************************************************************
* Definition/implementation of Map<LieGroup<Array<Scalar, 7, 1> > >
***************************************************************************/

// we don't need anything new


#endif
