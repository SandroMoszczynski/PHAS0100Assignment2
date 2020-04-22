/*=============================================================================
  PHAS0100ASSIGNMENT2: PHAS0100 Assignment 2 Social Force Model
  Copyright (c) University College London (UCL). All rights reserved.
  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.
  See LICENSE.txt in the top level directory for details.
  Based on prototype vec2d, pos2d, dir2d: 9 Mar 2020
      Author: Tim Spain
  Included in this project: 1 Apr 2020
      Author: Jim Dobson
=============================================================================*/

#include "sfmBasicTypes.h"

namespace sfm {

//=============================================================================*/
// vec2d definitions
//-----------------------------------------------------------------------------
vec2d vec2d::operator+(const vec2d &b) const {
  return vec2d(xx + b.xx, yy + b.yy);
}

//-----------------------------------------------------------------------------
vec2d vec2d::operator-(const vec2d &b) const {
  return vec2d(xx - b.xx, yy - b.yy);
}

//-----------------------------------------------------------------------------
double vec2d::operator*(const vec2d &b) const {
  return xx * b.xx + yy * b.yy;
}

//-----------------------------------------------------------------------------
vec2d vec2d::operator*(const double &c) const {
  return vec2d(c * xx, c * yy);
}

//=============================================================================*/
// pos2d definitions
//-----------------------------------------------------------------------------
// Move in a given direction from this position
pos2d pos2d::operator+(const dir2d &b) const {
  return this->displace(b);
}

//-----------------------------------------------------------------------------
// N.B. ignores periodic boundary conditions
dir2d pos2d::operator-(const pos2d &b) const {
  return pos2d::direction(b, *this);
}       
 
//-----------------------------------------------------------------------------
// N.B. ignores periodic boundary conditions
double pos2d::distance(const pos2d &b) const {
  dir2d d = this->direction(b);
  return sqrt(d.x() * d.x() + d.y() * d.y());
}       
        
//-----------------------------------------------------------------------------
// direction vector a - b, where 'this' is a
// N.B. ignores periodic boundary conditions
dir2d pos2d::direction(const pos2d& b) const {
  return direction(*this, b);
}

//-----------------------------------------------------------------------------
// direction vector from a to b
// N.B. ignores periodic boundary conditions
dir2d pos2d::direction(const pos2d &a, const pos2d &b) {
  double dx = b.x() - a.x();
  double dy = b.y() - a.y();
  return dir2d(dx, dy);
}

//-----------------------------------------------------------------------------
pos2d pos2d::displace(const dir2d &b) const {
  double nx = x() + b.x();
  double ny = y() + b.y();
  return pos2d(nx, ny);
}


//=============================================================================*/
// dir2d definitions
//-----------------------------------------------------------------------------
pos2d dir2d::operator+(const pos2d &b) const {
  return displace(b);
}

//-----------------------------------------------------------------------------
dir2d dir2d::operator+(const dir2d &b) const {
  return dir2d(x() + b.x(), y() + b.y());
}

//-----------------------------------------------------------------------------
dir2d dir2d::operator-(const dir2d &b) const {
  return dir2d(x() - b.x(), y() - b.y());
}

//-----------------------------------------------------------------------------
double dir2d::operator*(const dir2d &b) const {
  return x() * b.x() + y() * b.y();
}

//-----------------------------------------------------------------------------
dir2d dir2d::operator*(const double &s) const {
	return dir2d(x() * s, y() * s);
}

//-----------------------------------------------------------------------------
dir2d dir2d::operator-() const {
  return dir2d(-x(), -y());
}

//-----------------------------------------------------------------------------
pos2d dir2d::displace(const pos2d &b) const {
  return b.displace(*this);
}

//-----------------------------------------------------------------------------
dir2d dir2d::add(const dir2d &b) const {
  return *this + b;
}

//-----------------------------------------------------------------------------
dir2d dir2d::subtract(const dir2d &b) const {
  return *this - b;
}

//-----------------------------------------------------------------------------
double dir2d::scalar_product(const dir2d &b) const {
  return *this * b;
}

//-----------------------------------------------------------------------------
double dir2d::length() const {
  return sqrt(scalar_product(*this));
}

//-----------------------------------------------------------------------------
double dir2d::cos_angle(const dir2d &b) const {
  return scalar_product(b)/(length() * b.length());
}

//-----------------------------------------------------------------------------
void dir2d::negate( ) {
  *this = this->negative();
}

//-----------------------------------------------------------------------------
dir2d dir2d::negative( ) const {
  return dir2d(-x(), -y());
}

//-----------------------------------------------------------------------------
dir2d operator*(const double &s, const dir2d &v) {
	return v * s;
}
  
} // end namespace