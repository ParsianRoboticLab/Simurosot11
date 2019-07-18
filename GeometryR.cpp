/*
Copyright (c) 2000-2002, Jelle Kok, University of Amsterdam
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Amsterdam nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNE
R OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*! \file Geometry.C
<pre>
<b>File:</b>          Geometry.C
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       13/02/2001
<b>Last Revision:</b> $ID$
<b>Contents:</b>      class declarations of different geometry classes:<BR>
- VecPosition: representation of a point
- Line:        representation of a line
- Rectangle:   representation of a rectangle
- Circle:      representation of a circle
- Geometry:    different geometry methods

Furthermore it contains some goniometric functions to work with sine, cosine
and tangent functions using degrees and some utility functions to return
the maximum and the minimum of two values.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
12/02/2001       Jelle Kok       Initial version created
</pre>
*/
#include "stdafx.h"

#include "GeometryR.h"

#include <stdio.h>    // needed for sprintf


/*******************************************************************************************/
/** Following is Math functions ************************************************************/
/*******************************************************************************************/
/*! This function returns the Maths::Sign of a give double.
1 is positive, -1 is negative
\param d1 first parameter
\return the Maths::Sign of this double */
int Maths::Sign(double d1) {
	return (d1 > 0) ? 1 : -1;
}

double Maths::Limit(double d, double dMin, double dMax) {
	if (dMin > dMax) {
		double dTemp = dMin;
		dMin = dMax;
		dMax = dTemp;
	}
	
	if (d < dMin) d = dMin;
	if (d > dMax) d = dMax;
	return d;
}

/*! This function returns the maximum of two given doubles.
\param d1 first parameter
\param d2 second parameter
\return the maximum of these two parameters */
double Maths::Max(double d1, double d2) {
	return (d1 > d2) ? d1 : d2;
}

/*! This function returns the minimum of two given doubles.
\param d1 first parameter
\param d2 second parameter
\return the minimum of these two parameters */
double Maths::Min(double d1, double d2) {
	return (d1 < d2) ? d1 : d2;
}


/*! This function converts an angle in radians to the corresponding angle in
degrees.
\param x an angle in radians
\return the corresponding angle in degrees */
AngDeg Maths::Rad2Deg(AngRad x) {
	return (x * 180 / M_PI);
}

/*! This function converts an angle in degrees to the corresponding angle in
radians.
\param x an angle in degrees
\return the corresponding angle in radians */
AngRad Maths::Deg2Rad(AngDeg x) {
	return (x * M_PI / 180);
}

/*! This function returns the cosine of a given angle in degrees using the
built-in cosine function that works with angles in radians.
\param x an angle in degrees
\return the cosine of the given angle */
double Maths::cosDeg(AngDeg x) {
	return (cos(Deg2Rad(x)));
}

/*! This function returns the sine of a given angle in degrees using the
built-in sine function that works with angles in radians.
\param x an angle in degrees
\return the sine of the given angle */
double Maths::sinDeg(AngDeg x) {
	return (sin(Deg2Rad(x)));
}

/*! This function returns the tangent of a given angle in degrees using the
built-in tangent function that works with angles in radians.
\param x an angle in degrees
\return the tangent of the given angle */
double Maths::tanDeg(AngDeg x) {
	return (tan(Deg2Rad(x)));
}

/*! This function returns the principal value of the arc tangent of x in degrees
using the built-in arc tangent function which returns this value in radians.
\param x a double value
\return the arc tangent of the given value in degrees */
AngDeg Maths::atanDeg(double x) {
	return (Rad2Deg(atan(x)));
}

/*! This function returns the principal value of the arc tangent of y/x in
degrees using the signs of both arguments to determine the quadrant of the
return value. For this the built-in 'atan2' function is used which returns
this value in radians.
\param x a double value
\param y a double value
\return the arc tangent of y/x in degrees taking the signs of x and y into
account */
double Maths::atan2Deg(double x, double y) {
	if (fabs(x) < EPS && fabs(y) < EPS)
		return (0.0);
	
	return (Rad2Deg(atan2(x, y)));
}

/*! This function returns the principal value of the arc cosine of x in degrees
using the built-in arc cosine function which returns this value in radians.
\param x a double value
\return the arc cosine of the given value in degrees */
AngDeg Maths::acosDeg(double x) {
	if (x >= 1)
		return (0.0);
	else if (x <= -1)
		return (180.0);
	
	return (Rad2Deg(acos(x)));
}

/*! This function returns the principal value of the arc sine of x in degrees
using the built-in arc sine function which returns this value in radians.
\param x a double value
\return the arc sine of the given value in degrees */
AngDeg Maths::asinDeg(double x) {
	if (x >= 1)
		return (90.0);
	else if (x <= -1)
		return (-90.0);
	
	return (Rad2Deg(asin(x)));
}

/*********************************************************************************/



/*! This function returns a boolean value which indicates whether the value
'ang' (from interval [-180..180] lies in the interval [angMin..angMax].
Examples: IsAngInInterval( -100, 4, -150) returns false
IsAngInInterval(   45, 4, -150) returns true
\param ang angle that should be checked
\param angMin minimum angle in interval
\param angMax maximum angle in interval
\return boolean indicating whether ang lies in [angMin..angMax] */
bool IsAngInInterval(AngRad ang, AngRad angMin, AngRad angMax) {
	// convert all angles to interval 0..360
	if ((ang + 2 * M_PI) < 2 * M_PI) ang += 2 * M_PI;
	if ((angMin + 2 * M_PI) < 2 * M_PI) angMin += 2 * M_PI;
	if ((angMax + 2 * M_PI) < 2 * M_PI) angMax += 2 * M_PI;
	
	if (angMin < angMax) // 0 ---false-- angMin ---true-----angMax---false--360
		return angMin < ang && ang < angMax;
	else                  // 0 ---true--- angMax ---false----angMin---true---360
		return !(angMax < ang && ang < angMin);
}

/*! This method returns the bisector (average) of two angles. It deals
with the boundary problem, thus when 'angMin' equals 170 and 'angMax'
equals -100, -145 is returned.
\param angMin minimum angle [-PI,PI]
\param angMax maximum angle [-PI,PI]
\return average of angMin and angMax. */
AngRad GetBisectorTwoAngles(AngRad angMin, AngRad angMax) {
	// separate sine and cosine part to circumvent boundary problem
	return VecPosition::NormalizeAngle(
			atan2((sin(angMin) + sin(angMax)) / 2.0,
			      (cos(angMin) + cos(angMax)) / 2.0));
}

/******************************************************************************/
/********************   CLASS VECPOSITION   ***********************************/
/******************************************************************************/

/*! Constructor for the VecPosition claSS-> When the supplied Coordinate System
type equals CARTESIAN, the arguments x and y denote the x- and y-coordinates
of the new position. When it equals POLAR however, the arguments x and y
denote the polar coordinates of the new position; in this case x is thus
equal to the distance r from the origin and y is equal to the angle phi that
the polar vector Makes with the x-axis.
\param x the x-coordinate of the new position when cs == CARTESIAN; the
distance of the new position from the origin when cs = POLAR
\param y the y-coordinate of the new position when cs = CARTESIAN; the
angle that the polar vector Makes with the x-axis when cs = POLAR
\param cs a CoordSystemT indicating whether x and y denote cartesian
coordinates or polar coordinates
\return the VecPosition corresponding to the given arguments */
VecPosition::VecPosition(double x, double y, CoordSystemT cs) {
	SetVecPosition(x, y, cs);
}


dbPOINT VecPosition::GetdbPOINT() {
	dbPOINT p;
	p.x = m_x;
	p.y = m_y;
	return p;
}

/*! Overloaded version of unary minus operator for VecPositions. It returns the
negative VecPosition, i.e. both the x- and y-coordinates are multiplied by
-1. The current VecPosition itself is left unchanged.
\return a negated version of the current VecPosition */
VecPosition VecPosition::operator-() {
	return (VecPosition(-m_x, -m_y));
}

/*! Overloaded version of the binary plus operator for adding a given double
value to a VecPosition. The double value is added to both the x- and
y-coordinates of the current VecPosition. The current VecPosition itself is
left unchanged.
\param d a double value which has to be added to both the x- and
y-coordinates of the current VecPosition
\return the result of adding the given double value to the current
VecPosition */
VecPosition VecPosition::operator+(const double &d) {
	return (VecPosition(m_x + d, m_y + d));
}

/*! Overloaded version of the binary plus operator for VecPositions. It returns
the sum of the current VecPosition and the given VecPosition by adding their
x- and y-coordinates. The VecPositions themselves are left unchanged.
\param p a VecPosition
\return the sum of the current VecPosition and the given VecPosition */
VecPosition VecPosition::operator+(const VecPosition &p) {
	return (VecPosition(m_x + p.m_x, m_y + p.m_y));
}

/*! Overloaded version of the binary minus operator for subtracting a given
double value from a VecPosition. The double value is subtracted from both
the x- and y-coordinates of the current VecPosition. The current VecPosition
itself is left unchanged.
\param d a double value which has to be subtracted from both the x- and
y-coordinates of the current VecPosition
\return the result of subtracting the given double value from the current
VecPosition */
VecPosition VecPosition::operator-(const double &d) {
	return (VecPosition(m_x - d, m_y - d));
}

/*! Overloaded version of the binary minus operator for VecPositions. It returns
the difference between the current VecPosition and the given VecPosition by
subtracting their x- and y-coordinates. The VecPositions themselves are left
unchanged.
\param p a VecPosition
\return the difference between the current VecPosition and the given
VecPosition */
VecPosition VecPosition::operator-(const VecPosition &p) {
	return (VecPosition(m_x - p.m_x, m_y - p.m_y));
}

/*! Overloaded version of the multiplication operator for multiplying a
VecPosition by a given double value. Both the x- and y-coordinates of the
current VecPosition are multiplied by this value. The current VecPosition
itself is left unchanged.
\param d the multiplication factor
\return the result of multiplying the current VecPosition by the given
double value */
VecPosition VecPosition::operator*(const double &d) {
	return (VecPosition(m_x * d, m_y * d));
}

/*! Overloaded version of the multiplication operator for VecPositions. It
returns the product of the current VecPosition and the given VecPosition by
multiplying their x- and y-coordinates. The VecPositions themselves are left
unchanged.
\param p a VecPosition
\return the product of the current VecPosition and the given VecPosition */
VecPosition VecPosition::operator*(const VecPosition &p) {
	return (VecPosition(m_x * p.m_x, m_y * p.m_y));
}

/*! Overloaded version of the division operator for dividing a VecPosition by a
given double value. Both the x- and y-coordinates of the current VecPosition
are divided by this value. The current VecPosition itself is left unchanged.
\param d the division factor
\return the result of dividing the current VecPosition by the given double
value */
VecPosition VecPosition::operator/(const double &d) {
	return (VecPosition(m_x / d, m_y / d));
}

/*! Overloaded version of the division operator for VecPositions. It returns the
quotient of the current VecPosition and the given VecPosition by dividing
their x- and y-coordinates. The VecPositions themselves are left unchanged.
\param p a VecPosition
\return the quotient of the current VecPosition and the given VecPosition */
VecPosition VecPosition::operator/(const VecPosition &p) {
	return (VecPosition(m_x / p.m_x, m_y / p.m_y));
}

/*! Overloaded version of the assignment operator for assigning a given double
value to both the x- and y-coordinates of the current VecPosition. This
changes the current VecPosition itself.
\param d a double value which has to be assigned to both the x- and
y-coordinates of the current VecPosition */
void VecPosition::operator=(const double &d) {
	m_x = d;
	m_y = d;
}

/*! Overloaded version of the sum-assignment operator for VecPositions. It
returns the sum of the current VecPosition and the given VecPosition by
adding their x- and y-coordinates. This changes the current VecPosition
itself.
\param p a VecPosition which has to be added to the current VecPosition */
void VecPosition::operator+=(const VecPosition &p) {
	m_x += p.m_x;
	m_y += p.m_y;
}

/*! Overloaded version of the sum-assignment operator for adding a given double
value to a VecPosition. The double value is added to both the x- and
y-coordinates of the current VecPosition. This changes the current
VecPosition itself.
\param d a double value which has to be added to both the x- and
y-coordinates of the current VecPosition */
void VecPosition::operator+=(const double &d) {
	m_x += d;
	m_y += d;
}

/*! Overloaded version of the difference-assignment operator for VecPositions.
It returns the difference between the current VecPosition and the given
VecPosition by subtracting their x- and y-coordinates. This changes the
current VecPosition itself.
\param p a VecPosition which has to be subtracted from the current
VecPosition */
void VecPosition::operator-=(const VecPosition &p) {
	m_x -= p.m_x;
	m_y -= p.m_y;
}

/*! Overloaded version of the difference-assignment operator for subtracting a
given double value from a VecPosition. The double value is subtracted from
both the x- and y-coordinates of the current VecPosition. This changes the
current VecPosition itself.
\param d a double value which has to be subtracted from both the x- and
y-coordinates of the current VecPosition */
void VecPosition::operator-=(const double &d) {
	m_x -= d;
	m_y -= d;
}

/*! Overloaded version of the multiplication-assignment operator for
VecPositions. It returns the product of the current VecPosition and the
given VecPosition by multiplying their x- and y-coordinates. This changes
the current VecPosition itself.
\param p a VecPosition by which the current VecPosition has to be
multiplied */
void VecPosition::operator*=(const VecPosition &p) {
	m_x *= p.m_x;
	m_y *= p.m_y;
}

/*! Overloaded version of the multiplication-assignment operator for multiplying
a VecPosition by a given double value. Both the x- and y-coordinates of the
current VecPosition are multiplied by this value. This changes the current
VecPosition itself.
\param d a double value by which both the x- and y-coordinates of the
current VecPosition have to be multiplied */
void VecPosition::operator*=(const double &d) {
	m_x *= d;
	m_y *= d;
}

/*! Overloaded version of the division-assignment operator for VecPositions. It
returns the quotient of the current VecPosition and the given VecPosition by
dividing their x- and y-coordinates. This changes the current VecPosition
itself.
\param p a VecPosition by which the current VecPosition has to be divided */
void VecPosition::operator/=(const VecPosition &p) {
	m_x /= p.m_x;
	m_y /= p.m_y;
}

/*! Overloaded version of the division-assignment operator for dividing a
VecPosition by a given double value. Both the x- and y-coordinates of the
current VecPosition are divided by this value. This changes the current
VecPosition itself.
\param d a double value by which both the x- and y-coordinates of the
current VecPosition have to be divided */
void VecPosition::operator/=(const double &d) {
	m_x /= d;
	m_y /= d;
}

/*! Overloaded version of the inequality operator for VecPositions. It
determines whether the current VecPosition is unequal to the given
VecPosition by comparing their x- and y-coordinates.
\param p a VecPosition
\return true when either the x- or y-coordinates of the given VecPosition
and the current VecPosition are different; false otherwise */
bool VecPosition::operator!=(const VecPosition &p) {
	return ((fabs(m_x - p.m_x) > EPS) || (fabs(m_y - p.m_y) > EPS));
}

/*! Overloaded version of the inequality operator for comparing a VecPosition to
a double value. It determines whether either the x- or y-coordinate of the
current VecPosition is unequal to the given double value.
\param d a double value with which both the x- and y-coordinates of the
current VecPosition have to be compared.
\return true when either the x- or y-coordinate of the current VecPosition
is unequal to the given double value; false otherwise */
bool VecPosition::operator!=(const double &d) {
	return ((fabs(m_x - d) > EPS) || (fabs(m_y - d) > EPS));
}

/*! Overloaded version of the equality operator for VecPositions. It determines
whether the current VecPosition is equal to the given VecPosition by
comparing their x- and y-coordinates.
\param p a VecPosition
\return true when both the x- and y-coordinates of the given VecPosition and
the current VecPosition are equal; false otherwise */
bool VecPosition::operator==(const VecPosition &p) {
	return ((fabs(m_x - p.m_x) < EPS) && (fabs(m_y - p.m_y) < EPS));
}

/*! Overloaded version of the equality operator for comparing a VecPosition to a
double value. It determines whether both the x- and y-coordinates of the
current VecPosition are equal to the given double value.
\param d a double value with which both the x- and y-coordinates of the
current VecPosition have to be compared.
\return true when both the x- and y-coordinates of the current VecPosition
are equal to the given double value; false otherwise */
bool VecPosition::operator==(const double &d) {
	return ((fabs(m_x - d) < EPS) && (fabs(m_y - d) < EPS));
}


/*! Overloaded version of the C++ output operator for VecPositions. This
operator Makes it possible to use VecPositions in output statements (e.g.
cout << v). The x- and y-coordinates of the VecPosition are printed in the
format (x,y).
\param os output stream to which information should be written
\param v a VecPosition which must be printed
\return output stream containing (x,y) */
/*
::ostream& operator <<( ::ostream &os, VecPosition v )
{
return ( os << "( " << v.m_x << ", " << v.m_y << " )" );
}
*/

/*! This method writes the current VecPosition to standard output. It can also
print a polar representation of the current VecPosition.
\param cs a CoordSystemtT indicating whether a POLAR or CARTESIAN
representation of the current VecPosition should be printed */
void VecPosition::Show(CoordSystemT cs) {
	if (cs == CARTESIAN)
		printf("x = %5.2f, y = %5.2f", m_x, m_y);
	else
		printf("r = %5.2f, phi = %5.2f", GetMagnitude(), GetDirection());
}

void VecPosition::Print(FILE *file) {
	fprintf(file, "%2.2f\t%2.2f", m_x, m_y);
}


/*! This method writes the current VecPosition to a string. It can also write a
polar representation of the current VecPosition.
\param cs a CoordSystemtT indicating whether a POLAR or CARTESIAN
representation of the current VecPosition should be written
\return a string containing a polar or Cartesian representation of the
current VecPosition depending on the value of the boolean argument */
void VecPosition::ToString(char buf[], CoordSystemT cs) {
	if (cs == CARTESIAN)
		sprintf(buf, "( %5.2f, %5.2f )", GetX(), GetY());
	else
		sprintf(buf, "( r: %5.2f, phi: %5.2f )", GetMagnitude(), GetDirection());
}

/*! Set method for the x-coordinate of the current VecPosition.
\param dX a double value representing a new x-coordinate
\return a boolean indicating whether the Update was successful */
bool VecPosition::SetX(double dX) {
	m_x = dX;
	return (true);
}

/*! Get method for the x-coordinate of the current VecPosition.
\return the x-coordinate of the current VecPosition */
double VecPosition::GetX() const {
	return (m_x);
}

/*! Set method for the y-coordinate of the current VecPosition.
\param dY a double value representing a new y-coordinate
\return a boolean indicating whether the Update was successful */
bool VecPosition::SetY(double dY) {
	m_y = dY;
	return (true);
}

/*! Get method for the y-coordinate of the current VecPosition.
\return the y-coordinate of the current VecPosition */
double VecPosition::GetY() const {
	return (m_y);
}

/*! This method (re)Sets the coordinates of the current VecPosition. The given
coordinates can either be polar or Cartesian coordinates. This is indicated
by the value of the third argument.
\param dX a double value indicating either a new Cartesian x-coordinate when
cs=CARTESIAN or a new polar r-coordinate (distance) when cs=POLAR
\param dY a double value indicating either a new Cartesian y-coordinate when
cs=CARTESIAN or a new polar phi-coordinate (angle) when cs=POLAR
\param cs a CoordSystemT indicating whether x and y denote cartesian
coordinates or polar coordinates */
void VecPosition::SetVecPosition(double dX, double dY, CoordSystemT cs) {
	if (cs == CARTESIAN) {
		m_x = dX;
		m_y = dY;
	} else
		*this = GetVecPositionFromPolar(dX, dY);
}

/*! This method determines the distance between the current VecPosition and a
given VecPosition. This is equal to the magnitude (length) of the vector
connecting the two positions which is the difference vector between them.
\param p a Vecposition
\return the distance between the current VecPosition and the given
VecPosition */
double VecPosition::GetDistanceTo(const VecPosition p) {
	return ((*this - p).GetMagnitude());
}

/*! This method adjusts the coordinates of the current VecPosition in such a way
that the magnitude of the corresponding vector equals the double value which
is supplied as an argument. It thus scales the vector to a given length by
multiplying both the x- and y-coordinates by the quotient of the argument
and the current magnitude. This changes the VecPosition itself.
\param d a double value representing a new magnitude
\return the result of scaling the vector corresponding with the current
VecPosition to the given magnitude thus yielding a different VecPosition */
VecPosition VecPosition::SetMagnitude(double d) {
	if (GetMagnitude() > EPS)
		(*this) *= (d / GetMagnitude());
	
	return (*this);
}

/*! This method determines the magnitude (length) of the vector corresponding
with the current VecPosition using the formula of Pythagoras.
\return the length of the vector corresponding with the current
VecPosition */
double VecPosition::GetMagnitude() const {
	return (sqrt(m_x * m_x + m_y * m_y));
}

/*! This method determines the direction of the vector corresponding with the
current VecPosition (the phi-coordinate in polar representation) using the
arc tangent function. Note that the signs of x and y have to be taken into
account in order to determine the correct quadrant.
\return the direction in degrees of the vector corresponding with the
current VecPosition */
AngRad VecPosition::GetDirection() const {
	return (atan2(m_y, m_x));
}

/*! This method determines whether the current VecPosition is in front of a
given VecPosition, i.e. whether the x-coordinate of the current VecPosition
is larger than the x-coordinate of the given VecPosition.
\param p a VecPosition to which the current VecPosition must be compared
\return true when the current VecPosition is in front of the given
VecPosition; false otherwise */
bool VecPosition::IsRightOf(const VecPosition &p) {
	return m_x >= p.GetX();
}

/*! This method determines whether the x-coordinate of the current VecPosition
is in front of (i.e. larger than) a given double value.
\param d a double value to which the current x-coordinate must be compared
\return true when the current x-coordinate is in front of the given value;
false otherwise */
bool VecPosition::IsRightOf(const double &d) {
	return m_x >= d;
}

/*! This method determines whether the current VecPosition is behind a given
VecPosition, i.e. whether the x-coordinate of the current VecPosition is
smaller than the x-coordinate of the given VecPosition.
\param p a VecPosition to which the current VecPosition must be compared
\return true when the current VecPosition is behind the given VecPosition;
false otherwise */
bool VecPosition::IsLeftOf(const VecPosition &p) {
	return m_x <= p.GetX();
}

/*! This method determines whether the x-coordinate of the current VecPosition
is behind (i.e. smaller than) a given double value.
\param d a double value to which the current x-coordinate must be compared
\return true when the current x-coordinate is behind the given value; false
otherwise */
bool VecPosition::IsLeftOf(const double &d) {
	return m_x <= d;
}

/*! This method determines whether the current VecPosition is to the left of a
given VecPosition, i.e. whether the y-coordinate of the current VecPosition
is smaller than the y-coordinate of the given VecPosition.
\param p a VecPosition to which the current VecPosition must be compared
\return true when the current VecPosition is to the left of the given
VecPosition; false otherwise */
bool VecPosition::IsButtomOf(const VecPosition &p) {
	return m_y <= p.GetY();
}

/*! This method determines whether the y-coordinate of the current VecPosition
is to the left of (i.e. smaller than) a given double value.
\param d a double value to which the current y-coordinate must be compared
\return true when the current y-coordinate is to the left of the given
value; false otherwise */
bool VecPosition::IsButtomOf(const double &d) {
	return m_y <= d;
}

/*! This method determines whether the current VecPosition is to the right of a
given VecPosition, i.e. whether the y-coordinate of the current VecPosition
is larger than the y-coordinate of the given VecPosition.
\param p a VecPosition to which the current VecPosition must be compared
\return true when the current VecPosition is to the right of the given
VecPosition; false otherwise */
bool VecPosition::IsTopOf(const VecPosition &p) {
	return m_y >= p.GetY();
}

/*! This method determines whether the y-coordinate of the current VecPosition
is to the right of (i.e. larger than) a given double value.
\param d a double value to which the current y-coordinate must be compared
\return true when the current y-coordinate is to the right of the given
value; false otherwise */
bool VecPosition::IsTopOf(const double &d) {
	return m_y >= d;
}

/*! This method determines whether the current VecPosition is in between two
given VecPositions when looking in the x-direction, i.e. whether the current
VecPosition is in front of the first argument and behind the second.
\param p1 a VecPosition to which the current VecPosition must be compared
\param p2 a VecPosition to which the current VecPosition must be compared
\return true when the current VecPosition is in between the two given
VecPositions when looking in the x-direction; false otherwise */
bool VecPosition::IsBetweenX(const VecPosition &p1, const VecPosition &p2) {
	return IsRightOf(p1) && IsLeftOf(p2);
}

/*! This method determines whether the x-coordinate of the current VecPosition
is in between two given double values, i.e. whether the x-coordinate of the
current VecPosition is in front of the first argument and behind the second.
\param d1 a double value to which the current x-coordinate must be compared
\param d2 a double value to which the current x-coordinate must be compared
\return true when the current x-coordinate is in between the two given
values; false otherwise */
bool VecPosition::IsBetweenX(const double &d1, const double &d2) {
	return IsRightOf(d1) && IsLeftOf(d2);
}

/*! This method determines whether the current VecPosition is in between two
given VecPositions when looking in the y-direction, i.e. whether the current
VecPosition is to the right of the first argument and to the left of the
second.
\param p1 a VecPosition to which the current VecPosition must be compared
\param p2 a VecPosition to which the current VecPosition must be compared
\return true when the current VecPosition is in between the two given
VecPositions when looking in the y-direction; false otherwise */
bool VecPosition::IsBetweenY(const VecPosition &p1, const VecPosition &p2) {
	return IsTopOf(p1) && IsButtomOf(p2);
}

/*! This method determines whether the y-coordinate of the current VecPosition
is in between two given double values, i.e. whether the y-coordinate of the
current VecPosition is to the right of the first argument and to the left
of the second.
\param d1 a double value to which the current y-coordinate must be compared
\param d2 a double value to which the current y-coordinate must be compared
\return true when the current y-coordinate is in between the two given
values; false otherwise */
bool VecPosition::IsBetweenY(const double &d1, const double &d2) {
	return IsTopOf(d1) && IsButtomOf(d2);
}

/*! This method Normalizes a VecPosition by Setting the magnitude of the
corresponding vector to 1. This thus changes the VecPosition itself.
\return the result of normalizing the current VecPosition thus yielding a
different VecPosition */
VecPosition VecPosition::Normalize() {
	return (SetMagnitude(1.0));
}

/*! This method Rotates the vector corresponding to the current VecPosition over
a given angle thereby changing the current VecPosition itself. This is done
by calculating the polar coordinates of the current VecPosition and adding
the given angle to the phi-coordinate in the polar representation. The polar
coordinates are then converted back to Cartesian coordinates to obtain the
desired result.
\param angle an angle in degrees over which the vector corresponding to the
current VecPosition must be Rotated
\return the result of rotating the vector corresponding to the current
VecPosition over the given angle thus yielding a different VecPosition */
VecPosition VecPosition::Rotate(AngRad angle) {
	// determine the polar representation of the current VecPosition
	double dMag = this->GetMagnitude();
	double dNewDir = this->GetDirection() + angle;  // add rotation angle to phi
	SetVecPosition(dMag, dNewDir, POLAR);          // convert back to Cartesian
	return (*this);
}

/*! This method converts the coordinates of the current VecPosition (which are
represented in an global coordinate system with the origin at (0,0)) into
relative coordinates in a different coordinate system (e.g. relative to a
player). The new coordinate system is defined by the arguments to the
method. The relative coordinates are now obtained by aligning the relative
coordinate system with the global coordinate system using a translation to
Make both origins coincide followed by a rotation to align the axes.
\param origin the origin of the relative coordinate frame
\param ang the angle between the world frame and the relative frame
(reasoning from the world frame)
\return the result of converting the current global VecPosition into a
relative VecPosition */
VecPosition VecPosition::GlobalToRelative(VecPosition origin, AngRad ang) {
	// convert global coordinates into relative coordinates by aligning relative
	// frame and world frame. First perform translation to Make origins of both
	// frames coincide. Then perform rotation to Make axes of both frames coincide
	// (use negative angle since you Rotate relative frame to world frame).
	*this -= origin;
	return (Rotate(-ang));
}

/*! This method converts the coordinates of the current VecPosition (which are
represented in a relative coordinate system) into global coordinates in
the world frame (with origin at (0,0)). The relative coordinate system is
defined by the arguments to the method. The global coordinates are now
obtained by aligning the world frame with the relative frame using a
rotation to align the axes followed by a translation to Make both origins
coincide.
\param origin the origin of the relative coordinate frame
\param ang the angle between the world frame and the relative frame
(reasoning from the world frame)
\return the result of converting the current relative VecPosition into an
global VecPosition */
VecPosition VecPosition::RelativeToGlobal(VecPosition origin, AngRad ang) {
	// convert relative coordinates into global coordinates by aligning world
	// frame and relative frame. First perform rotation to Make axes of both
	// frames coincide (use positive angle since you Rotate world frame to
	// relative frame). Then perform translation to Make origins of both frames
	// coincide.
	Rotate(ang);
	*this += origin;
	return (*this);
}

/*! This method returns a VecPosition that lies somewhere on the vector between
the current VecPosition and a given VecPosition. The desired position is
specified by a given fraction of this vector (e.g. 0.5 means exactly in
the middle of the vector). The current VecPosition itself is left unchanged.
\param p a VecPosition which defines the vector to the current VecPosition
\param dFrac double representing the fraction of the connecting vector at
which the desired VecPosition lies.
\return the VecPosition which lies at fraction dFrac on the vector
connecting p and the current VecPosition */
VecPosition VecPosition::GetVecPositionOnLineFraction(VecPosition &p,
                                                      double dFrac) {
	// determine point on line that lies at fraction dFrac of whole line
	// example: this --- 0.25 ---------  p
	// formula: this + dFrac * ( p - this ) = this - dFrac * this + dFrac * p =
	//          ( 1 - dFrac ) * this + dFrac * p
	return ((*this) * (1.0 - dFrac) + (p * dFrac));
}

/*! This method converts a polar representation of a VecPosition into a
Cartesian representation.
\param dMag a double representing the polar r-coordinate, i.e. the distance
from the point to the origin
\param ang the angle that the polar vector Makes with the x-axis, i.e. the
polar phi-coordinate
\return the result of converting the given polar representation into a
Cartesian representation thus yielding a Cartesian VecPosition */
VecPosition VecPosition::GetVecPositionFromPolar(double dMag, AngRad ang) {
	// cos(phi) = x/r <=> x = r*cos(phi); sin(phi) = y/r <=> y = r*sin(phi)
	return (VecPosition(dMag * cos(ang), dMag * sin(ang)));
}

/*! This method Normalizes an angle. This means that the resulting angle lies
between -PI and PI degrees.
\param angle the angle which must be Normalized
\return the result of normalizing the given angle */
AngRad VecPosition::NormalizeAngle(AngRad angle) {
	while (angle > M_PI) angle -= 2 * M_PI;
	while (angle < -M_PI) angle += 2 * M_PI;
	
	return (angle);
}

AngRad VecPosition::NormalizeAngle2PI(AngRad angle) {
	while (angle > 2 * M_PI) angle -= 2 * M_PI;
	while (angle < 0) angle += 2 * M_PI;
	
	return (angle);
}


/******************************************************************************/
/*********************** CLASS GEOMETRY ***************************************/
/******************************************************************************/

/*! A geometric series is one in which there is a constant ratio between each
element and the one preceding it. This method determines the
length of a geometric series given its first element, the sum of the
elements in the series and the constant ratio between the elements.
Normally: s = a + ar + ar^2 + ...  + ar^n
Now: dSum = dFirst + dFirst*dRatio + dFirst*dRatio^2 + .. + dFist*dRatio^n
\param dFirst first term of the series
\param dRatio ratio with which the the first term is multiplied
\param dSum the total sum of all the serie
\return the length(n in above example) of the series */
double Geometry::GetLengthGeomSeries(double dFirst, double dRatio, double dSum) {
	// s = a + ar + ar^2 + .. + ar^n-1 and thus sr = ar + ar^2 + .. + ar^n
	// subtract: sr - s = - a + ar^n) =>  s(1-r)/a + 1 = r^n = temp
	// log r^n / n = n log r / log r = n = length
	double temp = (dSum * (dRatio - 1) / dFirst) + 1;
	if (temp <= 0)
		return -1.0;
	return log(temp) / log(dRatio);
}

/*! A geometric series is one in which there is a constant ratio between each
element and the one preceding it. This method determines the sum of a
geometric series given its first element, the ratio and the number of steps
in the series
Normally: s = a + ar + ar^2 + ...  + ar^n
Now: dSum = dFirst + dFirst*dRatio + ... + dFirst*dRatio^dSteps
\param dFirst first term of the series
\param dRatio ratio with which the the first term is multiplied
\param dSum the number of steps to be taken into account
\return the sum of the series */
double Geometry::GetSumGeomSeries(double dFirst, double dRatio, double dLength) {
	// s = a + ar + ar^2 + .. + ar^n-1 and thus sr = ar + ar^2 + .. + ar^n
	// subtract: s - sr = a - ar^n) =>  s = a(1-r^n)/(1-r)
	return dFirst * (1 - pow(dRatio, dLength)) / (1 - dRatio);
}

/*! A geometric series is one in which there is a constant ratio between each
element and the one preceding it. This method determines the sum of an
infinite geometric series given its first element and the constant ratio
between the elements. Note that such an infinite series will only converge
when 0<r<1.
Normally: s = a + ar + ar^2 + ar^3 + ....
Now: dSum = dFirst + dFirst*dRatio + dFirst*dRatio^2...
\param dFirst first term of the series
\param dRatio ratio with which the the first term is multiplied
\return the sum of the series */
double Geometry::GetSumInfGeomSeries(double dFirst, double dRatio) {
	
	// s = a(1-r^n)/(1-r) with n->inf and 0<r<1 => r^n = 0
	return dFirst / (1 - dRatio);
}

/*! A geometric series is one in which there is a constant ratio between each
element and the one preceding it. This method determines the first element
of a geometric series given its element, the ratio and the number of steps
in the series
Normally: s = a + ar + ar^2 + ...  + ar^n
Now: dSum = dFirst + dFirst*dRatio + ... + dFirst*dRatio^dSteps
\param dSum sum of the series
\param dRatio ratio with which the the first term is multiplied
\param dSum the number of steps to be taken into account
\return the first element (a) of a serie */
double Geometry::GetFirstGeomSeries(double dSum, double dRatio, double dLength) {
	// s = a + ar + ar^2 + .. + ar^n-1 and thus sr = ar + ar^2 + .. + ar^n
	// subtract: s - sr = a - ar^n) =>  s = a(1-r^n)/(1-r) => a = s*(1-r)/(1-r^n)
	return dSum * (1 - dRatio) / (1 - pow(dRatio, dLength));
}

/*! A geometric series is one in which there is a constant ratio between each
element and the one preceding it. This method determines the first element
of an infinite geometric series given its first element and the constant
ratio between the elements. Note that such an infinite series will only
converge when 0<r<1.
Normally: s = a + ar + ar^2 + ar^3 + ....
Now: dSum = dFirst + dFirst*dRatio + dFirst*dRatio^2...
\param dSum sum of the series
\param dRatio ratio with which the the first term is multiplied
\return the first term of the series */
double Geometry::GetFirstInfGeomSeries(double dSum, double dRatio) {
	
	// s = a(1-r^n)/(1-r) with r->inf and 0<r<1 => r^n = 0 => a = s ( 1 - r)
	return dSum * (1 - dRatio);
}

/*! This method performs the abc formula (Pythagoras' Theorem) on the given
parameters and puts the result in *s1 en *s2. It returns the number of
found coordinates.
\param a a parameter in abc formula
\param b b parameter in abc formula
\param c c parameter in abc formula
\param *s1 first result of abc formula
\param *s2 second result of abc formula
\return number of found x-coordinates */
int Geometry::AbcFormula(double a, double b, double c, double *s1, double *s2) {
	double dDiscr = b * b - 4 * a * c;       // discriminant is b^2 - 4*a*c
	if (fabs(dDiscr) < EPS)       // if discriminant = 0
	{
		*s1 = -b / (2 * a);              //  only one solution
		return 1;
	} else if (dDiscr < 0)               // if discriminant < 0
		return 0;                        //  no solutions
	else                               // if discriminant > 0
	{
		dDiscr = sqrt(dDiscr);           //  two solutions
		*s1 = (-b + dDiscr) / (2 * a);
		*s2 = (-b - dDiscr) / (2 * a);
		return 2;
	}
}

/******************************************************************************/
/********************** CLASS CIRCLE ******************************************/
/******************************************************************************/

/*! This is the constructor of a circle.
\param pos first point that defines the center of circle
\param dR the radius of the circle
\return circle with pos as center and radius as radius*/
Circle::Circle(VecPosition pos, double dR) {
	SetCircle(pos, dR);
}

/*! This is the constructor of a circle which initializes a circle with a
radius of zero. */
Circle::Circle() {
	SetCircle(VecPosition(-1000.0, -1000.0), 0);
}

/*! This method prints the circle information to the specified output stream
in the following format: "c: (c_x,c_y), r: rad" where (c_x,c_y) denotes
the center of the circle and rad the radius.
\param os output stream to which output is written. */
void Circle::Show() {
	printf("c:");
	m_posCenter.Show();
	printf(", r: %5.2f \n", m_dRadius);
}

/*! This method Sets the values of the circle.
\param pos new center of the circle
\param dR new radius of the circle
( > 0 )
\return bool indicating whether radius was Set */
bool Circle::SetCircle(VecPosition pos, double dR) {
	SetCenter(pos);
	return SetRadius(dR);
}

/*! This method Sets the radius of the circle.
\param dR new radius of the circle ( > 0 )
\return bool indicating whether radius was Set */
bool Circle::SetRadius(double dR) {
	if (dR > 0) {
		m_dRadius = dR;
		return true;
	} else {
		m_dRadius = 0.0;
		return false;
	}
}

/*! This method returns the radius of the circle.
\return radius of the circle */
double Circle::GetRadius() {
	return m_dRadius;
}

/*! This method Sets the center of the circle.
\param pos new center of the circle
\return bool indicating whether center was Set */
bool Circle::SetCenter(VecPosition pos) {
	m_posCenter = pos;
	return true;
}

/*! This method returns the center of the circle.
\return center of the circle */
VecPosition Circle::GetCenter() {
	return m_posCenter;
}

/*! This method returns the circumference of the circle.
\return circumference of the circle */
double Circle::GetCircumference() {
	return 2.0 * M_PI * GetRadius();
}

/*! This method returns the area inside the circle.
\return area inside the circle */
double Circle::GetArea() {
	return M_PI * GetRadius() * GetRadius();
}

/*! This method returns a boolean that indicates whether 'pos' is located inside
the circle.
\param pos position of which should be checked whether it is located in the
circle
\return bool indicating whether pos lies inside the circle */
bool Circle::IsInside(VecPosition pos) {
	return m_posCenter.GetDistanceTo(pos) < GetRadius();
}

/*! This method returns the two possible intersection points between two
circles. This method returns the number of solutions that were found.
\param c circle with which intersection should be found
\param p1 will be filled with first solution
\param p2 will be filled with second solution
\return number of solutions. */
int Circle::GetIntersectionPoints(Circle c, VecPosition *p1, VecPosition *p2) {
	double x0, y0, r0;
	double x1, y1, r1;
	
	x0 = GetCenter().GetX();
	y0 = GetCenter().GetY();
	r0 = GetRadius();
	x1 = c.GetCenter().GetX();
	y1 = c.GetCenter().GetY();
	r1 = c.GetRadius();
	
	double d, dx, dy, h, a, x, y, p2_x, p2_y;
	
	// first Calculate distance between two centers circles P0 and P1.
	dx = x1 - x0;
	dy = y1 - y0;
	d = sqrt(dx * dx + dy * dy);
	
	// Normalize differences
	dx /= d;
	dy /= d;
	
	// a is distance between p0 and point that is the intersection point P2
	// that intersects P0-P1 and the line that crosses the two intersection
	// points P3 and P4.
	// Define two triangles: P0,P2,P3 and P1,P2,P3.
	// with distances a, h, r0 and b, h, r1 with d = a + b
	// We know a^2 + h^2 = r0^2 and b^2 + h^2 = r1^2 which then gives
	// a^2 + r1^2 - b^2 = r0^2 with d = a + b ==> a^2 + r1^2 - (d-a)^2 = r0^2
	// ==> r0^2 + d^2 - r1^2 / 2*d
	a = (r0 * r0 + d * d - r1 * r1) / (2.0 * d);
	
	// h is then a^2 + h^2 = r0^2 ==> h = sqrt( r0^2 - a^2 )
	double arg = r0 * r0 - a * a;
	h = (arg > 0.0) ? sqrt(arg) : 0.0;
	
	// First Calculate P2
	p2_x = x0 + a * dx;
	p2_y = y0 + a * dy;
	
	// and finally the two intersection points
	x = p2_x - h * dy;
	y = p2_y + h * dx;
	p1->SetVecPosition(x, y);
	x = p2_x + h * dy;
	y = p2_y - h * dx;
	p2->SetVecPosition(x, y);
	
	return (arg < 0.0) ? 0 : ((arg == 0.0) ? 1 : 2);
}

/*! This method returns the size of the intersection area of two circles.
\param c circle with which intersection should be determined
\return size of the intersection areACT-> */
double Circle::GetIntersectionArea(Circle c) {
	VecPosition pos1, pos2, pos3;
	double d, h, dArea;
	AngDeg ang;
	
	d = GetCenter().GetDistanceTo(c.GetCenter()); // dist between two centers
	if (d > c.GetRadius() + GetRadius())           // larger than sum radii
		return 0.0;                                   // circles do not intersect
	if (d <= fabs(c.GetRadius() - GetRadius()))   // one totally in the other
	{
		double dR = Maths::Min(c.GetRadius(), GetRadius());// return area smallest circle
		return M_PI * dR * dR;
	}
	
	int iNrSol = GetIntersectionPoints(c, &pos1, &pos2);
	if (iNrSol != 2)
		return 0.0;
	
	// the intersection area of two circles can be divided into two segments:
	// left and right of the line between the two intersection points p1 and p2.
	// The outside area of each segment can be Calculated by taking the part
	// of the circle pie excluding the triangle from the center to the
	// two intersection points.
	// The pie equals pi*r^2 * rad(2*ang) / 2*pi = 0.5*rad(2*ang)*r^2 with ang
	// the angle between the center c of the circle and one of the two
	// intersection points. Thus the angle between c and p1 and c and p3 where
	// p3 is the point that lies halfway between p1 and p2.
	// This can be Calculated using ang = asin( d / r ) with d the distance
	// between p1 and p3 and r the radius of the circle.
	// The area of the triangle is 2*0.5*h*d.
	
	pos3 = pos1.GetVecPositionOnLineFraction(pos2, 0.5);
	d = pos1.GetDistanceTo(pos3);
	h = pos3.GetDistanceTo(GetCenter());
	ang = asin(d / GetRadius());
	
	dArea = ang * GetRadius() * GetRadius();
	dArea = dArea - d * h;
	
	// and now for the other segment the same story
	h = pos3.GetDistanceTo(c.GetCenter());
	ang = asin(d / c.GetRadius());
	dArea = dArea + ang * c.GetRadius() * c.GetRadius();
	dArea = dArea - d * h;
	
	return dArea;
}


/******************************************************************************/
/***********************  CLASS LINE *******************************************/
/******************************************************************************/

/*! This constructor creates a line by given the three coefficents of the line.
A line is specified by the formula ay + bx + c = 0.
\param dA a coefficients of the line
\param dB b coefficients of the line
\param dC c coefficients of the line */
Line::Line(double dA, double dB, double dC) {
	m_a = dA;
	m_b = dB;
	m_c = dC;
}

Line::Line() {
	m_a = m_b = m_c = 0;
}
/*! This function prints the line to the specified output stream in the
format y = ax + b.
\param os output stream to which output is written
\param l line that is written to output stream
\return output sream to which output is appended. */
/*
::ostream& operator <<(::ostream & os, Line l)
{
double a = l.GetACoefficient();
double b = l.GetBCoefficient();
double c = l.GetCCoefficient();

// ay + bx + c = 0 -> y = -b/a x - c/a
if( a == 0 )
os << "x = " << -c/b;
else
{
os << "y = ";
if( b != 0 )
os << -b/a << "x ";
if( c > 0 )
os << "- " <<  fabs(c/a);
else if( c < 0 )
os << "+ " <<  fabs(c/a);
}
return os;
}
*/

/*! This method prints the line information to the specified output stream.
\param os output stream to which output is written. */
void Line::Show() {
	// ay + bx + c = 0 -> y = -b/a x - c/a
	if (m_a == 0)
		printf("x = %5.2f", -m_c / m_b);
	else {
		printf("y = ");
		if (m_b != 0)
			printf("%5.2f x", -m_b / m_a);
		if (m_c > 0)
			printf("- %5.2f", fabs(m_c / m_a));
		else if (m_c < 0)
			printf("+ %5.2f", fabs(m_c / m_a));
	}
	printf("\n");
}

/*! This method returns the intersection point between the current Line and
the specified line.
\param line line with which the intersection should be Calculated.
\return VecPosition position that is the intersection point. */
VecPosition Line::GetIntersection(Line line) {
	VecPosition pos;
	double x, y;
	
	if (m_b / m_a == line.GetBCoefficient() / line.GetACoefficient()) // lines are parallel, no intersection
	{
		return pos;
	}
	if (m_a == 0)               // bx + c = 0 and a2*y + b2*x + c2 = 0 ==> x = -c/b
	{                          // Calculate x using the current line
		x = -m_c / m_b;                // and Calculate the y using the second line
		y = line.GetYGivenX(x);
	} else if (line.GetACoefficient() ==
	           0) {                         // ay + bx + c = 0 and b2*x + c2 = 0 ==> x = -c2/b2
		x = -line.GetCCoefficient() / line.GetBCoefficient(); // Calculate x using
		y = GetYGivenX(x);       // 2nd line and Calculate y using current line
	}
		// ay + bx + c = 0 and a2y + b2*x + c2 = 0
		// y = (-b2/a2)x - c2/a2
		// bx = -a*y - c =>  bx = -a*(-b2/a2)x -a*(-c2/a2) - c ==>
		// ==> a2*bx = a*b2*x + a*c2 - a2*c ==> x = (a*c2 - a2*c)/(a2*b - a*b2)
		// Calculate x using the above formula and the y using the current line
	else {
		x = (m_a * line.GetCCoefficient() - line.GetACoefficient() * m_c) /
		    (line.GetACoefficient() * m_b - m_a * line.GetBCoefficient());
		y = GetYGivenX(x);
	}
	
	return VecPosition(x, y);
}


/*! This method Calculates the intersection points between the current line
and the circle specified with as center 'posCenter' and radius 'dRadius'.
The number of solutions are returned and the corresponding points are put
in the third and fourth argument of the method
\param c circle with which intersection points should be found
\param posSolution1 first intersection (if any)
\param posSolution2 second intersection (if any) */
int Line::GetCircleIntersectionPoints(Circle circle,
                                      VecPosition *posSolution1, VecPosition *posSolution2) {
	int iSol;
	double dSol1, dSol2;
	double h = circle.GetCenter().GetX();
	double k = circle.GetCenter().GetY();
	
	// line:   x = -c/b (if a = 0)
	// circle: (x-h)^2 + (y-k)^2 = r^2, with h = center.x and k = center.y
	// fill in:(-c/b-h)^2 + y^2 -2ky + k^2 - r^2 = 0
	//         y^2 -2ky + (-c/b-h)^2 + k^2 - r^2 = 0
	// and determine solutions for y using abc-formula
	if (fabs(m_a) < EPS) {
		iSol = Geometry::AbcFormula(1, -2 * k, ((-m_c / m_b) - h) * ((-m_c / m_b) - h)
		                                       + k * k - circle.GetRadius() * circle.GetRadius(), &dSol1, &dSol2);
		posSolution1->SetVecPosition((-m_c / m_b), dSol1);
		posSolution2->SetVecPosition((-m_c / m_b), dSol2);
		return iSol;
	}
	
	// ay + bx + c = 0 => y = -b/a x - c/a, with da = -b/a and db = -c/a
	// circle: (x-h)^2 + (y-k)^2 = r^2, with h = center.x and k = center.y
	// fill in:x^2 -2hx + h^2 + (da*x-db)^2 -2k(da*x-db) + k^2 - r^2 = 0
	//         x^2 -2hx + h^2 + da^2*x^2 + 2da*db*x + db^2 -2k*da*x -2k*db
	//                                                         + k^2 - r^2 = 0
	//         (1+da^2)*x^2 + 2(da*db-h-k*da)*x + h2 + db^2  -2k*db + k^2 - r^2 = 0
	// and determine solutions for x using abc-formula
	// fill in x in original line equation to Get y coordinate
	double da = -m_b / m_a;
	double db = -m_c / m_a;
	
	double dA = 1 + da * da;
	double dB = 2 * (da * db - h - k * da);
	double dC = h * h + db * db - 2 * k * db + k * k - circle.GetRadius() * circle.GetRadius();
	
	iSol = Geometry::AbcFormula(dA, dB, dC, &dSol1, &dSol2);
	
	posSolution1->SetVecPosition(dSol1, da * dSol1 + db);
	posSolution2->SetVecPosition(dSol2, da * dSol2 + db);
	return iSol;
	
}

/*! This method returns the perpendicular line to a VecPosition. This is the line
between the specified position and the closest point on the line to this
position.
\param pos VecPosition point with which tangent line is Calculated.
\return Line line tangent to this position */
Line Line::GetPerpendicularLine(VecPosition pos) {
	// ay + bx + c = 0 -> y = (-b/a)x + (-c/a)
	// tangent: y = (a/b)*x + C1 -> by - ax + C2 = 0 => C2 = ax - by
	// with pos.y = y, pos.x = x
	return Line(m_b, -m_a, m_a * pos.GetX() - m_b * pos.GetY());
}

/*! This method returns the closest point on a line to a given position.
\param pos point to which closest point should be determined
\return VecPosition closest point on line to 'pos'. */
VecPosition Line::GetPointOnLineClosestTo(VecPosition pos) {
	Line l2 = GetPerpendicularLine(pos);  // Get tangent line
	return GetIntersection(l2);     // and intersection between the two lines
}

/*! This method returns the distance between a specified position and the
closest point on the given line.
\param pos position to which distance should be Calculated
\return double indicating the distance to the line. */
double Line::GetDistanceWithPoint(VecPosition pos) {
	return pos.GetDistanceTo(GetPointOnLineClosestTo(pos));
}

/*! This method determines whether the projection of a point on the current line
lies between two other points ('point1' and 'point2') that lie on the same
line.
\param pos point of which projection is checked.
\param point1 first point on line
\param point2 second point on line
\return true when projection of 'pos' lies between 'point1' and 'point2'.*/
bool Line::IsInBetween(VecPosition pos, VecPosition point1, VecPosition point2) {
	pos = GetPointOnLineClosestTo(pos); // Get closest point
	double dDist = point1.GetDistanceTo(point2); // Get distance between 2 pos
	
	// if the distance from both points to the projection is smaller than this
	// dist, the pos lies in between.
	return pos.GetDistanceTo(point1) <= dDist &&
	       pos.GetDistanceTo(point2) <= dDist;
}

/*! This method Calculates the y coordinate given the x coordinate
\param x coordinate
\return y coordinate on this line */
double Line::GetYGivenX(double x) {
	if (m_a == 0) {
		return 0;
	}
	// ay + bx + c = 0 ==> ay = -(b*x + c)/a
	return -(m_b * x + m_c) / m_a;
}

/*! This method Calculates the x coordinate given the x coordinate
\param y coordinate
\return x coordinate on this line */
double Line::GetXGivenY(double y) {
	if (m_b == 0) {
		return 0;
	}
	// ay + bx + c = 0 ==> bx = -(a*y + c)/a
	return -(m_a * y + m_c) / m_b;
}

/*! This method creates a line given two points.
\param pos1 first point
\param pos2 second point
\return line that passes through the two specified points. */
Line Line::MakeLineFromTwoPoints(VecPosition pos1, VecPosition pos2) {
	// 1*y + bx + c = 0 => y = -bx - c
	// with -b the direction coefficient (or slope)
	// and c = - y - bx
	double dA = 1.0, dB, dC;
	double dTemp = pos2.GetX() - pos1.GetX(); // determine the slope
	if (fabs(dTemp) < EPS) {
		// ay + bx + c = 0 with vertical slope=> a = 0, b = 1
		dA = 0.0;
		dB = 1.0;
	} else {
		// y = (-b)x -c with -b the slope of the line
		dA = 1.0;
		dB = -(pos2.GetY() - pos1.GetY()) / dTemp;
	}
	// ay + bx + c = 0 ==> c = -a*y - b*x
	dC = -dA * pos2.GetY() - dB * pos2.GetX();
	return Line(dA, dB, dC);
}

/*! This method creates a line given a position and an angle.
\param vec position through which the line passes
\param angle direction of the line.
\return line that goes through position 'vec' with angle 'angle'. */
Line Line::MakeLineFromPositionAndAngle(VecPosition vec, AngRad angle) {
	// Calculate point somewhat further in direction 'angle' and Make
	// line from these two points.
	return MakeLineFromTwoPoints(vec, vec + VecPosition(1, angle, POLAR));
}

/*! This method returns the a coefficient from the line ay + bx + c = 0.
\return a coefficient of the line. */
double Line::GetACoefficient() const {
	return m_a;
}

/*! This method returns the b coefficient from the line ay + bx + c = 0.
\return b coefficient of the line. */
double Line::GetBCoefficient() const {
	return m_b;
}

/*! This method returns the c coefficient from the line ay + bx + c = 0.
\return c coefficient of the line. */
double Line::GetCCoefficient() const {
	return m_c;
}

/*! This method returns the line point which is dDistance away from 
\return point pos1. if dDistance < 0, point is left of pos1. */
VecPosition Line::GetPointInLine(VecPosition pos1, double dDistance) {
	if (m_a == 0) {
		return VecPosition(0, 0);
	}
	double x = pos1.GetX() + dDistance / sqrt(1 + (m_b * m_b) / (m_a * m_a));
	
	// ay + bx + c = 0 ==> y = -(b*x + c)/a
	double y = -(m_b * x + m_c) / m_a;
	
	return VecPosition(x, y);
}

/******************************************************************************/
/********************** CLASS RECTANGLE ***************************************/
/******************************************************************************/

/*! This is the constructor of a Rectangle. Two points will be given. The
order does not matter as long as two opposite points are given (left
top and right bottom or right top and left bottom).
\param pos first point that defines corner of rectangle
\param pos2 second point that defines other corner of rectangle
\return rectangle with 'pos' and 'pos2' as opposite corners. */
Rect::Rect(VecPosition pos, VecPosition pos2) {
	SetRectanglePoints(pos, pos2);
}

/*! This method Sets the upper left and right bottom point of the current
rectangle.
\param pos first point that defines corner of rectangle
\param pos2 second point that defines other corner of rectangle */
void Rect::SetRectanglePoints(VecPosition pos1, VecPosition pos2) {
	m_posLeftTop.SetX(Maths::Min(pos1.GetX(), pos2.GetX()));
	m_posLeftTop.SetY(Maths::Min(pos1.GetY(), pos2.GetY()));
	m_posRightBottom.SetX(Maths::Max(pos1.GetX(), pos2.GetX()));
	m_posRightBottom.SetY(Maths::Max(pos1.GetY(), pos2.GetY()));
}

/*! This method prints the rectangle to the specified output stream in the
format rect( top_left_point, bottom_right_point ).
\param os output stream to which rectangle is written. */
void Rect::Show() {
	printf("rect(");
	m_posLeftTop.Show();
	printf(" ");
	m_posRightBottom.Show();
	printf(") \n");
}

/*! This method determines whether the given position lies inside the current
rectangle.
\param pos position which is checked whether it lies in rectangle
\return true when 'pos' lies in the rectangle, false otherwise */
bool Rect::IsInside(VecPosition pos) {
	return pos.IsBetweenX(m_posLeftTop.GetX(), m_posRightBottom.GetX()) &&
	       pos.IsBetweenY(m_posRightBottom.GetY(), m_posLeftTop.GetY());
	
}

/*! This method Sets the top left position of the rectangle
\param pos new top left position of the rectangle
\return true when Update was successful */
bool Rect::SetPosLeftTop(VecPosition pos) {
	m_posLeftTop = pos;
	return true;
}

/*! This method returns the top left position of the rectangle
\return top left position of the rectangle */
VecPosition Rect::GetPosLeftTop() {
	return m_posLeftTop;
}

/*! This method Sets the right bottom position of the rectangle
\param pos new right bottom position of the rectangle
\return true when Update was succesfull */
bool Rect::SetPosRightBottom(VecPosition pos) {
	m_posRightBottom = pos;
	return true;
}

/*! This method returns the right bottom position of the rectangle
\return top right bottom of the rectangle */
VecPosition Rect::GetPosRightBottom() {
	return m_posRightBottom;
}


