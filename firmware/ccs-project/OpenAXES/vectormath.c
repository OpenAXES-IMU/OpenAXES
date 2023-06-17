/*

@file  vectormath.c

@license  LGPL-3+

This file is part of the OpenAXES project, a wireless IMU.
Copyright 2023 Nils Stanislawski and Fritz Webering

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/


/** Gets the shortest arc quaternion to rotate this vector to the destination
    vector.
@remarks
    If you call this with a dest vector that is close to the inverse
    of this vector, we will rotate 180 degrees around the 'fallbackAxis'
    (if specified, or a generated axis if not) since in this case
    ANY axis of rotation is valid.
*/

#include "vectormath.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

#ifndef M_PI // math.h won't define M_PI under some circumstances
#define M_PI        3.14159265358979323846  /* pi */
#endif

Vector3 Matrix3x3_timesColumnVector(const Matrix3x3 mat, const Vector3 vec)
{
    const float *m = mat.d;
    const float *v = vec.d;
    return (Vector3) {{
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    }};
}

Matrix3x3 Matrix3x3_timesMatrix3x3(const Matrix3x3 mat1, const Matrix3x3 mat2)
{
    const float *m1 = mat1.d;
    const float *m2 = mat2.d;
    return (Matrix3x3) {{
    // result(row1)
        m1[0]*m2[0]  +  m1[1]*m2[3]  +  m1[2]*m2[6], // m1(row1) * m2(col1)
        m1[0]*m2[1]  +  m1[1]*m2[4]  +  m1[2]*m2[7], // m1(row1) * m2(col2)
        m1[0]*m2[2]  +  m1[1]*m2[5]  +  m1[2]*m2[8], // m1(row1) * m2(col3)
    // result(row2)
        m1[3]*m2[0]  +  m1[4]*m2[3]  +  m1[5]*m2[6], // m1(row2) * m2(col1)
        m1[3]*m2[1]  +  m1[4]*m2[4]  +  m1[5]*m2[7], // m1(row2) * m2(col2)
        m1[3]*m2[2]  +  m1[4]*m2[5]  +  m1[5]*m2[8], // m1(row2) * m2(col3)
    // result(row3)
        m1[6]*m2[0]  +  m1[7]*m2[3]  +  m1[8]*m2[6], // m1(row3) * m2(col1)
        m1[6]*m2[1]  +  m1[7]*m2[4]  +  m1[8]*m2[7], // m1(row3) * m2(col2)
        m1[6]*m2[2]  +  m1[7]*m2[5]  +  m1[8]*m2[8], // m1(row3) * m2(col3)
    }};
}

void Vector3_increment(Vector3 *_this, const Vector3 *increment)
{
    _this->x += increment->x;
    _this->y += increment->y;
    _this->z += increment->z;
}

Vector3 Vector3_add(Vector3 a, Vector3 b)
{
    return (Vector3) {{
        a.x + b.x,
        a.y + b.y,
        a.z + b.z,
    }};
}

Vector3 Vector3_mul(Vector3 a, float factor)
{
    return (Vector3) {{
        a.x * factor,
        a.y * factor,
        a.z * factor,
    }};
}

float Vector3_dotProduct(const Vector3 *v0, const Vector3 *v1)
{
    return v0->x * v1-> x +  v0->y * v1-> y +  v0->z * v1-> z;
}

Vector3 Vector3_crossProduct( const Vector3 *v0, const Vector3 *v1 )
{
    Vector3 result = {{
        v0->y * v1->z - v0->z * v1->y,
        v0->z * v1->x - v0->x * v1->z,
        v0->x * v1->y - v0->y * v1->x
    }};
    return result;
}

void Vector3_normalize(Vector3 *v)
{
    float len = sqrtf(Vector3_dotProduct(v, v));
    v->x = v->x / len;
    v->y = v->y / len;
    v->z = v->z / len;
}

bool Vector3_isZeroLength(const Vector3 *v)
{
    float length_squared = Vector3_dotProduct(v, v);
    return (length_squared < (1e-06 * 1e-06));
}

Vector3 Vector3_limit(Vector3 a, float limit)
{
    return (Vector3) {{
        a.x > limit ? limit : (a.x < -limit ? -limit : a.x),
        a.y > limit ? limit : (a.y < -limit ? -limit : a.y),
        a.z > limit ? limit : (a.z < -limit ? -limit : a.z),
    }};
}

Vector3 Vector3_round_int16(Vector3 a)
{
    a = Vector3_limit(a, INT16_MAX);
    return (Vector3) {{
        roundf(a.x), roundf(a.y), roundf(a.z)
    }};
}

void Quaternion_fromAngleAxis(Quaternion *this, float angle_rad, const Vector3 *axis)
{
    // assert:  axis[] is unit length
    //
    // The quaternion representing the rotation is
    //   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)

    float halfAngle = 0.5*angle_rad;
    float sinHalfAngle = sinf(halfAngle);
    this->w = cosf(halfAngle);
    this->x = sinHalfAngle * axis->x;
    this->y = sinHalfAngle * axis->y;
    this->z = sinHalfAngle * axis->z;
}

void Quaternion_normalize(Quaternion *q)
{
    float len = sqrtf(q->w * q->w + q->x * q-> x +  q->y * q-> y +  q->z * q-> z);
    q->w = q->w / len;
    q->x = q->x / len;
    q->y = q->y / len;
    q->z = q->z / len;
}

Quaternion Quaternion_mul(Quaternion q, float factor)
{
    return (Quaternion) {{
        q.w * factor,
        q.x * factor,
        q.y * factor,
        q.z * factor,
    }};
}

void getRotationBetweenVectors(Quaternion *q, const Vector3 *from, const Vector3 *to)
{
    // Adapted to C from https://github.com/ehsan/ogre/blob/master/OgreMain/include/OgreVector3.h
    // Based on Stan Melax's article in Game Programming Gems
    // Copy, since cannot modify local
    Vector3 v0 = *from;
    Vector3 v1 = *to;
    Vector3_normalize(&v0);
    Vector3_normalize(&v1);

    float d = Vector3_dotProduct(&v0, &v1);
    // If dot == 1, vectors are the same
    if (d >= (1.0f - 1e-6f))
    {
        q->w = 1;
        q->x = q->y = q->z = 0;
    }
    else if (d < (1e-6f - 1.0f))
    {
        // Generate an axis
        Vector3 X = {{1, 0, 0}};
        Vector3 axis = Vector3_crossProduct(&X, from);
        if (Vector3_isZeroLength(&axis)) { // pick another if colinear
            Vector3 Y = {{0, 1, 0}};
            axis = Vector3_crossProduct(&Y, from);
        }
        Vector3_normalize(&axis);
        Quaternion_fromAngleAxis(q, M_PI, &axis);
    }
    else
    {
        float s = sqrtf( (1+d)*2 );
        float inverse = 1 / s;

        Vector3 cross = Vector3_crossProduct(&v0, &v1);

        q->x = cross.x * inverse;
        q->y = cross.y * inverse;
        q->z = cross.z * inverse;
        q->w = s * 0.5f;
        Quaternion_normalize(q);
    }
}

