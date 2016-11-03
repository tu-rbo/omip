// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_ROTATION3D_H
#define EIGEN_LGSM_ROTATION3D_H

/***********************************************************************************
* There are no need for new constructor or accessor. Some typedefs will do the job.
************************************************************************************/

/** single precision 3D Rotation type */
typedef LieGroup<Quaternion<float> > Rotation3f;
/** double precision 3D Rotation type */
typedef LieGroup<Quaternion<double> > Rotation3d;

/** Map an unaligned array of single precision scalar as a 3D Rotation */
typedef Map<LieGroup<Quaternion<float> >, 0>         Rotation3Mapf;
/** Map an unaligned array of double precision scalar as a 3D Rotation */
typedef Map<LieGroup<Quaternion<double> >, 0>        Rotation3Mapd;
/** Map a 16-bits aligned array of double precision scalars as a 3D Rotation */
typedef Map<LieGroup<Quaternion<float> >, Aligned>   Rotation3MapAlignedf;
/** Map a 16-bits aligned array of double precision scalars as a 3D Rotation */
typedef Map<LieGroup<Quaternion<double> >, Aligned>  Rotation3MapAlignedd;

#endif
