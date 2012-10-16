// *****************************************************************
// This file is part of the IVT - Integrating Vision Toolkit.
//
// Copyright (C) 2004-2008 Pedram Azad, Chair Prof. Dillmann (IAIM),
// Institute for Computer Science and Engineering,
// University of Karlsruhe. All rights reserved.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public
// License along with this program; if not, write to the Free
// Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
// Boston, MA 02110-1301, USA.
// *****************************************************************

// *****************************************************************
// Filename:  Math3d.cpp
// Copyright: Pedram Azad, Chair Prof. Dillmann (IAIM),
//            Institute for Computer Science and Engineering (CSE),
//            University of Karlsruhe. All rights reserved.
// Author:    Pedram Azad
// Date:      2004
// *****************************************************************


// *****************************************************************
// includes
// *****************************************************************

#include "object_categorization/Math3d.h"

#include <math.h>




Vec3d Math3d::zero_vec = { 0, 0, 0 };
Mat3d Math3d::unit_mat = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
Mat3d Math3d::zero_mat = { 0, 0, 0, 0, 0, 0, 0, 0 ,0 };




void Math3d::SetVec(Vec3d &vec, float x, float y, float z)
{
	vec.x = x;
	vec.y = y;
	vec.z = z;
}

void Math3d::SetVec(Vec3d &vec, const Vec3d &sourceVector)
{
	vec.x = sourceVector.x;
	vec.y = sourceVector.y;
	vec.z = sourceVector.z;
}

void Math3d::SetMat(Mat3d &matrix, float r1, float r2, float r3, float r4, float r5, float r6, float r7, float r8, float r9)
{
	matrix.r1 = r1;
	matrix.r2 = r2;
	matrix.r3 = r3;
	matrix.r4 = r4;
	matrix.r5 = r5;
	matrix.r6 = r6;
	matrix.r7 = r7;
	matrix.r8 = r8;
	matrix.r9 = r9;
}

void Math3d::SetMat(Mat3d &matrix, const Mat3d &sourceMatrix)
{
	matrix.r1 = sourceMatrix.r1;
	matrix.r2 = sourceMatrix.r2;
	matrix.r3 = sourceMatrix.r3;
	matrix.r4 = sourceMatrix.r4;
	matrix.r5 = sourceMatrix.r5;
	matrix.r6 = sourceMatrix.r6;
	matrix.r7 = sourceMatrix.r7;
	matrix.r8 = sourceMatrix.r8;
	matrix.r9 = sourceMatrix.r9;
}

void Math3d::SetRotationMat(Mat3d &matrix, const Vec3d &rotation)
{
	const float alpha = rotation.x;
	const float beta = rotation.y;
	const float gamma = rotation.z;

	const float sinfalpha = sinf(alpha);
	const float cosfalpha = cosf(alpha);
	const float sinfbeta = sinf(beta);
	const float cosfbeta = cosf(beta);
	const float sinfgamma = sinf(gamma);
	const float cosfgamma = cosf(gamma);

	matrix.r1 = cosfbeta * cosfgamma;
	matrix.r2 = - cosfbeta * sinfgamma;
	matrix.r3 = sinfbeta;
	matrix.r4 = cosfalpha * sinfgamma + sinfalpha * sinfbeta * cosfgamma;
	matrix.r5 = cosfalpha * cosfgamma - sinfalpha * sinfbeta * sinfgamma;
	matrix.r6 = - sinfalpha * cosfbeta;
	matrix.r7 = sinfalpha * sinfgamma - cosfalpha * sinfbeta * cosfgamma;
	matrix.r8 = sinfalpha * cosfgamma + cosfalpha * sinfbeta * sinfgamma;
	matrix.r9 = cosfalpha * cosfbeta;
}

void Math3d::SetRotationMat(Mat3d &matrix, float alpha, float beta, float gamma)
{
	const float sinfalpha = sinf(alpha);
	const float cosfalpha = cosf(alpha);
	const float sinfbeta = sinf(beta);
	const float cosfbeta = cosf(beta);
	const float sinfgamma = sinf(gamma);
	const float cosfgamma = cosf(gamma);

	matrix.r1 = cosfbeta * cosfgamma;
	matrix.r2 = - cosfbeta * sinfgamma;
	matrix.r3 = sinfbeta;
	matrix.r4 = cosfalpha * sinfgamma + sinfalpha * sinfbeta * cosfgamma;
	matrix.r5 = cosfalpha * cosfgamma - sinfalpha * sinfbeta * sinfgamma;
	matrix.r6 = - sinfalpha * cosfbeta;
	matrix.r7 = sinfalpha * sinfgamma - cosfalpha * sinfbeta * cosfgamma;
	matrix.r8 = sinfalpha * cosfgamma + cosfalpha * sinfbeta * sinfgamma;
	matrix.r9 = cosfalpha * cosfbeta;
}

void Math3d::SetRotationMatYZX(Mat3d &matrix, const Vec3d &rotation)
{
	Mat3d temp;
	
	SetRotationMatY(matrix, rotation.y);
	
	SetRotationMatZ(temp, rotation.z);
	MulMatMat(temp, matrix, matrix);
	
	SetRotationMatX(temp, rotation.x);
	MulMatMat(temp, matrix, matrix);
}

void Math3d::SetRotationMatX(Mat3d &matrix, float theta)
{
	matrix.r1 = 1;
	matrix.r2 = matrix.r3 = matrix.r4 = matrix.r7 = 0;
	matrix.r5 = matrix.r9 = cosf(theta);
	matrix.r6 = matrix.r8 = sinf(theta);
	matrix.r6 = -matrix.r6;
}

void Math3d::SetRotationMatY(Mat3d &matrix, float theta)
{
	matrix.r5 = 1;
	matrix.r2 = matrix.r4 = matrix.r6 = matrix.r8 = 0;
	matrix.r1 = matrix.r9 = cosf(theta);
	matrix.r3 = matrix.r7 = sinf(theta);
	matrix.r7 = -matrix.r7;
}

void Math3d::SetRotationMatZ(Mat3d &matrix, float theta)
{
	matrix.r9 = 1;
	matrix.r3 = matrix.r6 = matrix.r7 = matrix.r8 = 0;
	matrix.r1 = matrix.r5 = cosf(theta);
	matrix.r2 = matrix.r4 = sinf(theta);
	matrix.r2 = -matrix.r2;
}

void Math3d::SetRotationMat(Mat3d &matrix, const Vec3d &axis, float theta)
{
	const float length = Length(axis);
	const float v1 = axis.x / length;
	const float v2 = axis.y / length;
	const float v3 = axis.z / length;

	const float t1 = cosf(theta);
	const float t2 = 1 - t1;
	const float t3 = v1 * v1;
	const float t6 = t2 * v1;
	const float t7 = t6 * v2;
	const float t8 = sinf(theta);
	const float t9 = t8 * v3;
	const float t11 = t6 * v3;
	const float t12 = t8 * v2;
	const float t15 = v2 * v2;
	const float t19 = t2 * v2 * v3;
	const float t20 = t8 * v1;
	const float t24 = v3 * v3;

	matrix.r1 = t1 + t2 * t3;
	matrix.r2 = t7 - t9;
	matrix.r3 = t11 + t12;
	matrix.r4 = t7 + t9;
	matrix.r5 = t1 + t2 * t15;
	matrix.r6 = t19 - t20;
	matrix.r7 = t11 - t12;
	matrix.r8 = t19 + t20;
	matrix.r9 = t1 + t2 * t24;
}

void Math3d::SetRotationMatAxis(Mat3d &matrix, const Vec3d &axis, float theta)
{
	const float length = Length(axis);
	const float x = axis.x / length;
	const float y = axis.y / length;
	const float z = axis.z / length;
	
	const float s = sinf(theta);
	const float c = cosf(theta);
	const float t = 1.0f - c;   
	
	matrix.r1 = t * x * x + c;
	matrix.r2 = t * x * y - s * z;
	matrix.r3 = t * x * z + s * y;	
	matrix.r4 = t * x * y + s * z;
	matrix.r5 = t * y * y + c;
	matrix.r6 = t * y * z - s * x;
	matrix.r7 = t * x * z - s * y;
	matrix.r8 = t * y * z + s * x;
	matrix.r9 = t * z * z + c;
}


void Math3d::MulMatVec(const Mat3d &matrix, const Vec3d &vec, Vec3d &result)
{
	const float x = vec.x;
	const float y = vec.y;
	const float z = vec.z;

 	result.x = matrix.r1 * x + matrix.r2 * y + matrix.r3 * z;
	result.y = matrix.r4 * x + matrix.r5 * y + matrix.r6 * z;
	result.z = matrix.r7 * x + matrix.r8 * y + matrix.r9 * z;
}

void Math3d::MulMatVec(const Mat3d &matrix, const Vec3d &vector1, const Vec3d &vector2, Vec3d &result)
{
	const float x = vector1.x;
	const float y = vector1.y;
	const float z = vector1.z;

	result.x = matrix.r1 * x + matrix.r2 * y + matrix.r3 * z + vector2.x;
	result.y = matrix.r4 * x + matrix.r5 * y + matrix.r6 * z + vector2.y;
	result.z = matrix.r7 * x + matrix.r8 * y + matrix.r9 * z + vector2.z;
}

void Math3d::MulMatMat(const Mat3d &matrix1, const Mat3d &matrix2, Mat3d &result)
{
	const float x1 = matrix1.r1 * matrix2.r1 + matrix1.r2 * matrix2.r4 + matrix1.r3 * matrix2.r7;
	const float x2 = matrix1.r1 * matrix2.r2 + matrix1.r2 * matrix2.r5 + matrix1.r3 * matrix2.r8;
	const float x3 = matrix1.r1 * matrix2.r3 + matrix1.r2 * matrix2.r6 + matrix1.r3 * matrix2.r9;
	const float x4 = matrix1.r4 * matrix2.r1 + matrix1.r5 * matrix2.r4 + matrix1.r6 * matrix2.r7;
	const float x5 = matrix1.r4 * matrix2.r2 + matrix1.r5 * matrix2.r5 + matrix1.r6 * matrix2.r8;
	const float x6 = matrix1.r4 * matrix2.r3 + matrix1.r5 * matrix2.r6 + matrix1.r6 * matrix2.r9;
	const float x7 = matrix1.r7 * matrix2.r1 + matrix1.r8 * matrix2.r4 + matrix1.r9 * matrix2.r7;
	const float x8 = matrix1.r7 * matrix2.r2 + matrix1.r8 * matrix2.r5 + matrix1.r9 * matrix2.r8;
	const float x9 = matrix1.r7 * matrix2.r3 + matrix1.r8 * matrix2.r6 + matrix1.r9 * matrix2.r9;
	
	result.r1 = x1;
	result.r2 = x2;
	result.r3 = x3;
	result.r4 = x4;
	result.r5 = x5;
	result.r6 = x6;
	result.r7 = x7;
	result.r8 = x8;
	result.r9 = x9;
}

void Math3d::MulVecTransposedVec(const Vec3d &vector1, const Vec3d &vector2, Mat3d &result)
{
	result.r1 = vector1.x * vector2.x;
	result.r2 = vector1.x * vector2.y;
	result.r3 = vector1.x * vector2.z;
	result.r4 = vector1.y * vector2.x;
	result.r5 = vector1.y * vector2.y;
	result.r6 = vector1.y * vector2.z;
	result.r7 = vector1.z * vector2.x;
	result.r8 = vector1.z * vector2.y;
	result.r9 = vector1.z * vector2.z;
}


void Math3d::AddToVec(Vec3d &vec, const Vec3d &vectorToAdd)
{
	vec.x += vectorToAdd.x;
	vec.y += vectorToAdd.y;
	vec.z += vectorToAdd.z;
}

void Math3d::SubtractFromVec(Vec3d &vec, const Vec3d &vectorToSubtract)
{
	vec.x -= vectorToSubtract.x;
	vec.y -= vectorToSubtract.y;
	vec.z -= vectorToSubtract.z;
}

void Math3d::AddVecVec(const Vec3d &vector1, const Vec3d &vector2, Vec3d &result)
{
	result.x = vector1.x + vector2.x;
	result.y = vector1.y + vector2.y;
	result.z = vector1.z + vector2.z;
}

void Math3d::MulVecScalar(const Vec3d &vec, float scalar, Vec3d &result)
{
	result.x = scalar * vec.x;
	result.y = scalar * vec.y;
	result.z = scalar * vec.z;
}

void Math3d::MulMatScalar(const Mat3d &matrix, float scalar, Mat3d &result)
{
	result.r1 = scalar * matrix.r1;
	result.r2 = scalar * matrix.r2;
	result.r3 = scalar * matrix.r3;
	result.r4 = scalar * matrix.r4;
	result.r5 = scalar * matrix.r5;
	result.r6 = scalar * matrix.r6;
	result.r7 = scalar * matrix.r7;
	result.r8 = scalar * matrix.r8;
	result.r9 = scalar * matrix.r9;
}

void Math3d::SubtractVecVec(const Vec3d &vector1, const Vec3d &vector2, Vec3d &result)
{
	result.x = vector1.x - vector2.x;
	result.y = vector1.y - vector2.y;
	result.z = vector1.z - vector2.z;
}


void Math3d::RotateVec(const Vec3d &vec, const Vec3d &rotation, Vec3d &result)
{
    Mat3d matrix;
	SetRotationMat(matrix, rotation);
	MulMatVec(matrix, vec, result);
}

void Math3d::TransformVec(const Vec3d &vec, const Vec3d &rotation, const Vec3d &translation, Vec3d &result)
{
    Mat3d matrix;
	SetRotationMat(matrix, rotation);
	MulMatVec(matrix, vec, translation, result);
}

void Math3d::RotateVecYZX(const Vec3d &vec, const Vec3d &rotation, Vec3d &result)
{
    Mat3d matrix;
	SetRotationMatYZX(matrix, rotation);
	MulMatVec(matrix, vec, result);
}

void Math3d::TransformVecYZX(const Vec3d &vec, const Vec3d &rotation, const Vec3d &translation, Vec3d &result)
{
    Mat3d matrix;
	SetRotationMatYZX(matrix, rotation);
	MulMatVec(matrix, vec, translation, result);
}


float Math3d::ScalarProduct(const Vec3d &vector1, const Vec3d &vector2)
{
	return vector1.x * vector2.x + vector1.y * vector2.y +  vector1.z * vector2.z;
}

void Math3d::CrossProduct(const Vec3d &vector1, const Vec3d &vector2, Vec3d &result)
{
	const float x = vector1.y * vector2.z - vector1.z * vector2.y;
	const float y = vector1.z * vector2.x - vector1.x * vector2.z;
	result.z = vector1.x * vector2.y - vector1.y * vector2.x;
	result.x = x;
	result.y = y;
}

void Math3d::NormalizeVec(Vec3d &vec)
{
	const float length = sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
	
	vec.x /= length;
	vec.y /= length;
	vec.z /= length;
}

float Math3d::Length(const Vec3d &vec)
{
	return sqrtf(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

float Math3d::SquaredLength(const Vec3d &vec)
{
	return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}

float Math3d::Distance(const Vec3d &vector1, const Vec3d &vector2)
{
	const float x1 = vector1.x - vector2.x;
	const float x2 = vector1.y - vector2.y;
	const float x3 = vector1.z - vector2.z;

	return sqrtf(x1 * x1 + x2 * x2 + x3 * x3);
}

float Math3d::SquaredDistance(const Vec3d &vector1, const Vec3d &vector2)
{
	const float x1 = vector1.x - vector2.x;
	const float x2 = vector1.y - vector2.y;
	const float x3 = vector1.z - vector2.z;

	return x1 * x1 + x2 * x2 + x3 * x3;
}

float Math3d::Angle(const Vec3d &vector1, const Vec3d &vector2)
{
	const float sp = vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
	const float l1 = sqrtf(vector1.x * vector1.x + vector1.y * vector1.y + vector1.z * vector1.z);
	const float l2 = sqrtf(vector2.x * vector2.x + vector2.y * vector2.y + vector2.z * vector2.z);
	
	// added this. In some cases angle was numerically unstable
	float r = sp / (l1 * l2);
	if (r > 1.0) r = 1.0;
	if (r < -1.0) r = -1.0;
	return acosf(r);
}

float Math3d::EvaluateForm(const Vec3d &matrix1, const Mat3d &matrix2)
{
	const float t0 = matrix1.x * matrix2.r1 + matrix1.y * matrix2.r4 + matrix1.z * matrix2.r7;
	const float t1 = matrix1.x * matrix2.r2 + matrix1.y * matrix2.r5 + matrix1.z * matrix2.r8;
	const float t2 = matrix1.x * matrix2.r3 + matrix1.y * matrix2.r6 + matrix1.z * matrix2.r9;

	return t0 * matrix1.x + t1 * matrix1.y + t2 * matrix1.z;
}

void Math3d::Transpose(const Mat3d &matrix, Mat3d &result)
{
	float temp;
	
	result.r1 = matrix.r1;
	result.r5 = matrix.r5;
	result.r9 = matrix.r9;
	
	temp = matrix.r4;
	result.r4 = matrix.r2;
	result.r2 = temp;
	
	temp = matrix.r3;
	result.r3 = matrix.r7;
	result.r7 = temp;
	
	temp = matrix.r6;
	result.r6 = matrix.r8;	
	result.r8 = temp;
	
}

void Math3d::Invert(const Mat3d &matrix, Mat3d &result)
{
	const float a = matrix.r1;
	const float b = matrix.r2;
	const float c = matrix.r3;
	const float d = matrix.r4;
	const float e = matrix.r5;
	const float f = matrix.r6;
	const float g = matrix.r7;
	const float h = matrix.r8;
	const float i = matrix.r9;

	float det_inverse = 1 / (-c * e * g + b * f * g + c * d * h - a * f * h - b * d * i + a * e * i);

	result.r1 = (-f * h + e * i) * det_inverse;
	result.r2 = (c * h - b * i) * det_inverse;
	result.r3 = (-c * e + b * f) * det_inverse;
	result.r4 = (f * g - d * i) * det_inverse;
	result.r5 = (-c * g + a * i) * det_inverse;
	result.r6 = (c * d - a * f) * det_inverse;
	result.r7 = (-e * g + d * h) * det_inverse;
	result.r8 = (b * g - a * h) * det_inverse;
	result.r9 = (-b * d + a * e) * det_inverse;
}

void Math3d::AddMatMat(const Mat3d &matrix1, const Mat3d &matrix2, Mat3d &matrix)
{
	matrix.r1 = matrix1.r1 + matrix2.r1;
	matrix.r2 = matrix1.r2 + matrix2.r2;
	matrix.r3 = matrix1.r3 + matrix2.r3;
	matrix.r4 = matrix1.r4 + matrix2.r4;
	matrix.r5 = matrix1.r5 + matrix2.r5;
	matrix.r6 = matrix1.r6 + matrix2.r6;
	matrix.r7 = matrix1.r7 + matrix2.r7;
	matrix.r8 = matrix1.r8 + matrix2.r8;
	matrix.r9 = matrix1.r9 + matrix2.r9;
}

void Math3d::AddToMat(Mat3d &matrix, const Mat3d &matrixToAdd)
{
	matrix.r1 += matrixToAdd.r1;
	matrix.r2 += matrixToAdd.r2;
	matrix.r3 += matrixToAdd.r3;
	matrix.r4 += matrixToAdd.r4;
	matrix.r5 += matrixToAdd.r5;
	matrix.r6 += matrixToAdd.r6;
	matrix.r7 += matrixToAdd.r7;
	matrix.r8 += matrixToAdd.r8;
	matrix.r9 += matrixToAdd.r9;
}

void Math3d::SubtractMatMat(const Mat3d &matrix1, const Mat3d &matrix2, Mat3d &result)
{
	result.r1 = matrix1.r1 - matrix2.r1;
	result.r2 = matrix1.r2 - matrix2.r2;
	result.r3 = matrix1.r3 - matrix2.r3;
	result.r4 = matrix1.r4 - matrix2.r4;
	result.r5 = matrix1.r5 - matrix2.r5;
	result.r6 = matrix1.r6 - matrix2.r6;
	result.r7 = matrix1.r7 - matrix2.r7;
	result.r8 = matrix1.r8 - matrix2.r8;
	result.r9 = matrix1.r9 - matrix2.r9;
}

float Math3d::Det(const Mat3d &matrix)
{
	return matrix.r1 * matrix.r5 * matrix.r9 
		+ matrix.r2 * matrix.r6 * matrix.r7
		+ matrix.r3 * matrix.r4 * matrix.r8
		- matrix.r3 * matrix.r5 * matrix.r7
		- matrix.r1 * matrix.r6 * matrix.r8
		- matrix.r2 * matrix.r4 * matrix.r9;
}


void Math3d::SetTransformation(Transformation3d &transformation, const Vec3d &rotation, const Vec3d &translation)
{
	Math3d::SetRotationMat(transformation.rotation, rotation);
	Math3d::SetVec(transformation.translation, translation);
}

void Math3d::SetTransformation(Transformation3d &transformation, const Transformation3d &sourceTransformation)
{
	Math3d::SetMat(transformation.rotation, sourceTransformation.rotation);
	Math3d::SetVec(transformation.translation, sourceTransformation.translation);
}

void Math3d::Invert(const Transformation3d &input, Transformation3d &result)
{
	Math3d::Invert(input.rotation, result.rotation);
	Math3d::MulMatVec(result.rotation, input.translation, result.translation);
	Math3d::MulVecScalar(result.translation, -1, result.translation);
}

void Math3d::MulTransTrans(const Transformation3d &transformation1, const Transformation3d &transformation2, Transformation3d &result)
{
	Math3d::MulMatVec(transformation1.rotation, transformation2.translation, transformation1.translation, result.translation);
	Math3d::MulMatMat(transformation1.rotation, transformation2.rotation, result.rotation);
}

void Math3d::MulTransVec(const Transformation3d &transformation, const Vec3d &vec, Vec3d &result)
{
	Math3d::MulMatVec(transformation.rotation, vec, transformation.translation, result);
}


void Math3d::MulQuatQuat(const Quaternion &quat1, const Quaternion &quat2, Quaternion &result)
{
	Vec3d v;
	
	CrossProduct(quat1.v, quat2.v, v);
	v.x += quat1.w * quat2.v.x + quat1.w * quat2.v.x;
	v.y += quat1.w * quat2.v.y + quat1.w * quat2.v.y;
	v.z += quat1.w * quat2.v.z + quat1.w * quat2.v.z;

	result.w = quat1.w * quat2.w - ScalarProduct(quat1.v, quat2.v);
	SetVec(result.v, v);
}


void Math3d::RotateVecQuaternion(const Vec3d &vec, const Vec3d &axis, float theta, Vec3d &result)
{
	/*const float st = sinf(theta * 0.5f);
	const float ct = cosf(theta * 0.5f);

	Quaternion a, q, r;
	Vec3d u;

	// u = normalized axis vector
	SetVec(u, axis);
	NormalizeVec(u);

	// set a
	SetVec(a.v, vec);
	a.w = 0;

	// set q
	q.v.x = st * u.x;
	q.v.y = st * u.y;
	q.v.z = st * u.z;
	q.w = ct;

	// calculate q * a
	MulQuatQuat(q, a, r);
	
	// calculate conjugate quaternion of q
	q.v.x = -q.v.x;
	q.v.y = -q.v.y;
	q.v.z = -q.v.z;

	// calculate q * a * q^
	MulQuatQuat(r, q, r);*/

	const float length = Length(axis);
	const float a = axis.x / length;
	const float b = axis.y / length;
	const float c = axis.z / length;
	const float d = theta;

	const float v1 = vec.x;
	const float v2 = vec.y;
	const float v3 = vec.z;

	const float t2 = a * b;
	const float t3 = a * c;
	const float t4 = a * d;
	const float t5 = -b * b;
	const float t6 = b * c;
	const float t7 = b * d;
	const float t8 = -c * c;
	const float t9 = c * d;
	const float t10 = -d * d;

	result.x = 2 * ((t8 + t10) * v1 + (t6 - t4) * v2 + (t3 + t7) * v3 ) + v1;
	result.y = 2 * ((t4 + t6) * v1 + (t5 + t10) * v2 + (t9 - t2) * v3 ) + v2;
	result.z = 2 * ((t7 - t3) * v1 + (t2 +  t9) * v2 + (t5 + t8) * v3 ) + v3;
}

void Math3d::RotateVecAngleAxis(const Vec3d &vec, const Vec3d &axis, float theta, Vec3d &result)
{
	Mat3d m;
	SetRotationMatAxis(m, axis, theta);
	MulMatVec(m, vec, result);
}

float Math3d::Angle(const Vec3d &vector1, const Vec3d &vector2, const Vec3d &axis)
{
	Vec3d u1, u2, n, temp;
	Mat3d testMatrix;

	Math3d::SetVec(n, axis);
	Math3d::NormalizeVec(n);

	Math3d::SetVec(u1, vector1);
	Math3d::MulVecScalar(n, Math3d::ScalarProduct(u1, n), temp);
	Math3d::SubtractFromVec(u1, temp);
	Math3d::NormalizeVec(u1);

	Math3d::SetVec(u2, vector2);
	Math3d::MulVecScalar(n, Math3d::ScalarProduct(u2, n), temp);
	Math3d::SubtractFromVec(u2, temp);
	Math3d::NormalizeVec(u2);
	
	// test which one of the two solutions is the right one
	Math3d::SetRotationMatAxis(testMatrix, n, Math3d::Angle(u1, u2));
	Math3d::MulMatVec(testMatrix, u1, temp);
	const float d1 = Math3d::Distance(temp, u2);

	Math3d::SetRotationMatAxis(testMatrix, n, -Math3d::Angle(u1, u2));
	Math3d::MulMatVec(testMatrix, u1, temp);
	const float d2 = Math3d::Distance(temp, u2);
	
	return d1 < d2 ? Math3d::Angle(u1, u2) : -Math3d::Angle(u1, u2);
}

void Math3d::GetAxisAndAngle(const Mat3d &R, Vec3d &axis, float &angle)
{
	const float x = R.r8 - R.r6;
	const float y = R.r3 - R.r7;
	const float z = R.r4 - R.r2;
	
	const float r = sqrtf(x * x + y * y + z * z);
	const float t = R.r1 + R.r5 + R.r9;
	
	angle = atan2(r, t - 1);
	
	Math3d::SetVec(axis, x, y, z);
}
