// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2013 CEA LIST (DIASI/LSI) <xde-support@saxifrage.cea.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef EIGEN_LGSM_LIE_MACROS_H
#define EIGEN_LGSM_LIE_MACROS_H

#define LIE_INHERIT_MATRIX_BASE(r, c)\
  inline Index rows() const { return r;} \
  inline Index cols() const { return c;} \
  \
  inline Scalar& coeffRef(Index row, Index col) { return this->get().coeffRef(row, col); } \
  inline const Scalar& coeff(Index row, Index col) const { return this->get().coeff(row, col); } \
  inline Scalar& coeffRef(Index index) { return this->get().coeffRef(index); } \
  inline const Scalar& coeff(Index index) const { return this->get().coeff(index); } \
  \
  template<int LoadMode> inline PacketScalar packet(Index index) const { return derived().get().template packet<LoadMode> (index);} \
  template<int LoadMode> inline PacketScalar packet(Index row, Index col) const { return derived().get().template packet<LoadMode> (row, col);} \
  template<int LoadMode> inline void writePacket(Index row, Index col, const PacketScalar& x) { derived().get().template writePacket<LoadMode>(row, col, x);} \
  template<int LoadMode> inline void writePacket(Index index, const PacketScalar& x) { derived().get().template writePacket<LoadMode>(index, x);} \
  \
  inline Index innerStride() const { return 1; } \
  inline Index outerStride() const { return this->innerSize(); } \
  \
  EIGEN_STRONG_INLINE const Scalar* data() const { return get().data(); } \
  EIGEN_STRONG_INLINE Scalar* data() { return get().data(); } \

#endif
