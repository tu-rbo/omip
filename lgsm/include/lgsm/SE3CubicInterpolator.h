// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_GROUP_SE3_CUBIC_INTERPOLATOR_H
#define EIGEN_LGSM_GROUP_SE3_CUBIC_INTERPOLATOR_H


/***************************************************************************************************
* Needed improvement :

- severals interpolator implemented through standard or static derivation
- implementing generic interpolation algorithm through template class (problem with limit conditions)

*****************************************************************************************************/

template<typename Scalar> class SE3CubicInterpolator{
public:
  typedef std::vector<Displacement<Scalar>, aligned_allocator<Displacement<Scalar> > > StdVectorDisplacement;
  typedef std::vector<Twist<Scalar>, aligned_allocator<Twist<Scalar> > > StdVectorTwist;

  SE3CubicInterpolator(){}

  void setControlPoint(const StdVectorDisplacement& controlPoints, const StdVectorTwist& controlVelocities, const std::vector<Scalar>& t);

  //template<class DisplDerived, class TwistDerived>
  //void Interpolate(DisplacementBase<DisplDerived>& pos, TwistBase<TwistDerived>& vel, const Scalar t) const;
  void Interpolate(Displacement<Scalar>& pos, Twist<Scalar>& vel, const Scalar time) const;
protected:
  std::vector<Scalar> t;
  Displacement<Scalar> H1;
  StdVectorTwist ksi;
  StdVectorTwist bi, ci, di; // revert order
  size_t n;
};

template<typename Scalar>
void SE3CubicInterpolator<Scalar>::setControlPoint(const StdVectorDisplacement& controlPoints, const StdVectorTwist& controlVelocities, const std::vector<Scalar>& ti){ // check if ctrlpts, velocities & time are equal and > 1 
  t = ti;
  n = t.size();
  H1 = controlPoints[0];

  //[XXX] this function assumes the object was just built. We have to clear the vectors if it's not the case. Can we implement a way to add points to an existing spline ?
  ksi.clear();
  ksi.reserve(n);
  bi.clear();
  bi.reserve(n-1); 
  ci.clear();
  ci.reserve(n); 
  di.clear();
  di.reserve(n-1);


  StdVectorDisplacement ctrlPts;  ctrlPts.reserve(n);
  StdVectorTwist alpha;           alpha.reserve(n);
  std::vector<Scalar> step;       step.reserve(n - 1);
  std::vector<Scalar> li;         li.reserve(n);
  std::vector<Scalar> mui;        mui.reserve(n);
  StdVectorTwist zi;              zi.reserve(n);

  Displacement<Scalar> offset = H1.inverse();

  // cubic spline : http://en.wikipedia.org/wiki/Spline_%28mathematics%29#Algorithm_for_computing_Clamped_Cubic_Splines

  for(typename StdVectorDisplacement::const_iterator iter = controlPoints.begin(); iter != controlPoints.end(); iter++){
    ctrlPts.push_back(offset*(*iter));
    ksi.push_back(ctrlPts.back().log()); // [XXX]PlainObject !
  }

  for(unsigned int i = 0; i < n - 1; i++) // n-1
    step.push_back(t[i+1] - t[i]);

  Twist<Scalar> dksi1 = ksi[0].dexp().inverse() * controlVelocities[0];
  alpha.push_back(3*((ksi[1] - ksi[0])/step[0] - dksi1));

  for(unsigned int i = 1; i < n - 1; i++){
    alpha.push_back(3*((ksi[i+1] - ksi[i])/step[i] - (ksi[i] - ksi[i-1])/step[i-1]));
  }

  Twist<Scalar> test = ksi.back();
  Twist<Scalar> tvel = controlVelocities.back();

  Matrix<Scalar, 6, 6> tm = ksi.back().dexp().inverse();

  Twist<Scalar> tdk = tm*tvel;

  Twist<Scalar> dksin = ksi.back().dexp().inverse() * controlVelocities.back();
  alpha.push_back(3*(dksin-(ksi.back() - ksi[n-2])/step.back()));


  li.push_back(2*step[0]);
  mui.push_back(0.5);
  zi.push_back(alpha[0]/li[0]);

  for(unsigned int i = 1; i < n - 1; i++){
    li.push_back(2*(t[i+1]-t[i-1]) - step[i-1]*mui[i-1]);
    mui.push_back(step[i]/li[i]);
    zi.push_back((alpha[i]-step[i-1]*zi[i-1])/li[i]);
  }

  li.push_back(step[n-2]*(2 - mui[n-2]));
  zi.push_back((alpha.back()-step[n-2]*zi[n-2])/li.back());

  ci.push_back(zi.back());

  for(unsigned int i = 1; i < n; i++){
    ci.push_back(zi[n-i-1] - mui[n-i-1]*ci[i-1]);
    bi.push_back((ksi[n-i]-ksi[n-i-1])/step[n-i-1]-step[n-i-1]*(ci[i-1]+2*ci[i])/3);
    di.push_back((ci[i-1]-ci[i])/3/step[n-i-1]);
  }
}

template<typename Scalar>
//template<class DisplDerived, class TwistDerived>
//void SE3CubicInterpolator<Scalar>::Interpolate(DisplacementBase<DisplDerived>& pos, TwistBase<TwistDerived>& vel, const Scalar time) const { 
void SE3CubicInterpolator<Scalar>::Interpolate(Displacement<Scalar>& pos, Twist<Scalar>& vel, const Scalar time) const { 
  assert(time > t[0] || time < t[n-1]); // check in release ?

  size_t k;
  for(size_t i = 0; i < n-1; i++){
    if(t[i] <= time)
      k = i;
  }

  if(k == n) k = n-1;

  Scalar dt = time-t[k];
  Scalar dt2 = dt*dt;

  Twist<Scalar> b = bi[n-2-k];
  Twist<Scalar> c = ci[n-1-k]; 
  Twist<Scalar> d = di[n-2-k];

  Twist<Scalar> ksik = ksi[k] + bi[n-2-k]*dt + ci[n-1-k]*dt2 + di[n-2-k]*dt2*dt; // c is longer by one element
  Twist<Scalar> dkisk = bi[n-2-k] + 2*ci[n-1-k]*dt + 3*di[n-2-k]*dt2;

  vel = ksik.dexp()*dkisk;
  pos = H1*ksik.exp();
}

#endif
