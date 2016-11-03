// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_TORQUE_H
#define EIGEN_LGSM_TORQUE_H

/***********************************************************************************
* There are no need for new constructor or accessor. Some typedefs will do the job.
************************************************************************************/

/** single precision angular velocity type */
typedef LieAlgebraDual<Matrix<float, 3, 1> > Torquef;
/** double precision angular velocity type */
typedef LieAlgebraDual<Matrix<double, 3, 1> > Torqued;

/** Map an unaligned array of single precision scalar as an angular velocity */
typedef Map<LieAlgebraDual<Matrix<float, 3, 1> >, 0>         TorqueMapf;
/** Map an unaligned array of double precision scalar as an angular velocity */
typedef Map<LieAlgebraDual<Matrix<double, 3, 1> >, 0>        TorqueMapd;
/** Map a 16-bits aligned array of double precision scalars an angular velocity */
typedef Map<LieAlgebraDual<Matrix<float, 3, 1> >, Aligned>  TorqueMapAlignedf;
/** Map a 16-bits aligned array of double precision scalars an angular velocity */
typedef Map<LieAlgebraDual<Matrix<double, 3, 1> >, Aligned>  TorqueMapAlignedd;

#endif
