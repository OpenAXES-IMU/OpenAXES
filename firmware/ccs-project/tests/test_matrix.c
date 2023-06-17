/*

@file  test_matrix.c

@license  LGPL-3+

This file is part of the OpenAXES project, a wireless IMU.
# Copyright 2023 Nils Stanislawski and Fritz Webering

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

#include "vectormath.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>

#define EPSILON 1e-8f
#define FLOAT_EQUAL(x, y) (fabs((x)-(y)) < (EPSILON))

void print_matrix(Matrix3x3* m)
{
    printf("%f, %f, %f,\n%f, %f, %f,\n%f, %f, %f,\n",
        m->d[0], m->d[1], m->d[2], m->d[3], m->d[4], m->d[5], m->d[6], m->d[7], m->d[8]);
}

void print_vector(Vector3* v)
{
    printf("%f, %f, %f,\n", v->d[0], v->d[1], v->d[2]);
}

void test_matrix_vector_mult()
{
    Matrix3x3 m1 = {{
        0.13702877, 0.5923936 , 0.5688447,
        0.49817324, 0.53613967, 0.7541049,
        0.8884016 , 0.984218  , 0.42166713
    }};
    Vector3 v = {{ 0.28714377, 0.5276601 , 0.7277246 }};
    Vector3 expected = {{ 0.7658917, 0.9747275, 1.081289 }};

    Vector3 result = Matrix3x3_timesColumnVector(m1, v);

    //print_matrix(result);
    //print_matrix(expected);

    assert(FLOAT_EQUAL(result.d[0], expected.d[0]));
    assert(FLOAT_EQUAL(result.d[1], expected.d[1]));
    assert(FLOAT_EQUAL(result.d[2], expected.d[2]));
}

void test_matrix_mult()
{
    Matrix3x3 m1 = {{
        0.13702877, 0.5923936 , 0.5688447,
        0.49817324, 0.53613967, 0.7541049,
        0.8884016 , 0.984218  , 0.42166713
    }};
    Matrix3x3 m2 = {{
        0.28714377, 0.5276601 , 0.7277246 ,
        0.9645512 , 0.49480975, 0.84997636,
        0.9474738 , 0.3483395 , 0.78529584
    }};
    Matrix3x3 expected = {{
        1.1497064, 0.5635778, 1.0499511,
        1.3746762, 0.7908378, 1.4104345,
        1.6039463, 1.102658 , 1.8142072
    }};

    Matrix3x3 result = Matrix3x3_timesMatrix3x3(m1, m2);

    //print_matrix(result);
    //print_matrix(expected);

    assert(FLOAT_EQUAL(result.d[0], expected.d[0]));
    assert(FLOAT_EQUAL(result.d[1], expected.d[1]));
    assert(FLOAT_EQUAL(result.d[2], expected.d[2]));
    assert(FLOAT_EQUAL(result.d[3], expected.d[3]));
    assert(FLOAT_EQUAL(result.d[4], expected.d[4]));
    assert(FLOAT_EQUAL(result.d[5], expected.d[5]));
    assert(FLOAT_EQUAL(result.d[6], expected.d[6]));
    assert(FLOAT_EQUAL(result.d[7], expected.d[7]));
    assert(FLOAT_EQUAL(result.d[8], expected.d[8]));
}

int main() {
    test_matrix_mult();
    test_matrix_vector_mult();
}
