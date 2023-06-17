/*

@file  vectormath.h

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

#ifndef APPLICATION_VECTORMATH_H_
#define APPLICATION_VECTORMATH_H_

#include <stdbool.h>

typedef union {
    struct { float w, x, y, z; };
    float d[4];
} Quaternion;

typedef union {
    struct { float x, y, z; };
    float d[3];
} Vector3;

typedef union {
    struct {
        float m11, m12, m13;
        float m21, m22, m23;
        float m31, m32, m33;
    };
    float d[9];
} Matrix3x3;


/**
 * Multiply 3x3 matrix with a column vector from the right: res = mat * vec (as column vector)
 *
 * res must not point to the same memory as vec!
 */
Vector3 Matrix3x3_timesColumnVector(const Matrix3x3 mat, const Vector3 vec);

/**
 * Multiply two 3x3 matrices res = mat1 * mat2
 *
 * res must not point to the same memory as mat1 or mat2!
 */
Matrix3x3 Matrix3x3_timesMatrix3x3(const Matrix3x3 mat1, const Matrix3x3 mat2);

/**
 * Add one vector to another, thereby modifying the target vector.
 */
void Vector3_increment(Vector3 *target, const Vector3 *increment);

/**
 * Add two vectors and return the sum.
 */
Vector3 Vector3_add(Vector3 a, Vector3 b);

/**
 * Add two vectors and return the sum.
 */
Vector3 Vector3_mul(Vector3 a, float factor);

/**
 * Limit the individual components so that their absolute values are less than or equal to limit.
 */
Vector3 Vector3_limit(Vector3 a, float limit);

/**
 * Round and limit the components to the int16_t value range.
 * The smallest allowed value is -INT16_MAX (-32767).
 * INT16_MIN is excluded to ensure conversion symmetry.
 */
Vector3 Vector3_round_int16(Vector3 a);


/** Gets the shortest arc quaternion to rotate this vector to the destination
    vector.
@remarks
    If you call this with a dest vector that is close to the inverse
    of this vector, we will rotate 180 degrees around the 'fallbackAxis'
    (if specified, or a generated axis if not) since in this case
    ANY axis of rotation is valid.
*/
void getRotationBetweenVectors(Quaternion *q, const Vector3 *from, const Vector3 *to);

void Quaternion_fromAngleAxis(Quaternion *this, float angle_rad, const Vector3 *axis);
void Quaternion_normalize(Quaternion *q);
Quaternion Quaternion_mul(Quaternion q, float factor);


float Vector3_dotProduct(const Vector3 *v0, const Vector3 *v1);
Vector3 Vector3_crossProduct( const Vector3 *v0, const Vector3 *v1 );
void Vector3_normalize(Vector3 *v);
bool Vector3_isZeroLength(const Vector3 *v);

#endif /* APPLICATION_VECTORMATH_H_ */
