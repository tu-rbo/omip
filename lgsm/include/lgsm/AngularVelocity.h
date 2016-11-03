// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_ANGULAR_VELOCITY_H
#define EIGEN_LGSM_ANGULAR_VELOCITY_H

/***********************************************************************************
* There are no need for new constructor or accessor. Some typedefs will do the job.
************************************************************************************/

/** single precision angular velocity type */
typedef LieAlgebra<Matrix<float, 3, 1> > AngularVelocityf;
/** double precision angular velocity type */
typedef LieAlgebra<Matrix<double, 3, 1> > AngularVelocityd;

/** Map an unaligned array of single precision scalar as an angular velocity */
typedef Map<LieAlgebra<Matrix<float, 3, 1> >, 0>         AngularVelocityMapf;
/** Map an unaligned array of double precision scalar as an angular velocity */
typedef Map<LieAlgebra<Matrix<double, 3, 1> >, 0>        AngularVelocityMapd;
/** Map a 16-bits aligned array of double precision scalars an angular velocity */
typedef Map<LieAlgebra<Matrix<float, 3, 1> >, Aligned>   AngularVelocityMapAlignedf;
/** Map a 16-bits aligned array of double precision scalars an angular velocity */
typedef Map<LieAlgebra<Matrix<double, 3, 1> >, Aligned>  AngularVelocityMapAlignedd;

#endif
